#!/usr/bin/env python3
"""x30_watchdog: a host's roslaunch under systemd's watchdog (tools/x30_host.sh start).

Starts the command (roslaunch), tells systemd READY=1 once the ROS master answers and every
critical node is registered and answering, then checks them every --period seconds and pings
systemd (WATCHDOG=1, with a STATUS line) while they hold. It ends the service with an error, for
systemd to start it again (Restart=on-failure), when:

- the master restarted (its /run_id changed): the nodes registered with the old one are lost;
- the master or a critical node stays away longer than --grace (a respawn takes seconds);
- the start takes longer than --startup;
- the command exits.

If the watchdog itself hangs it stops pinging, and systemd restarts the service after
WatchdogSec. SIGINT or SIGTERM stops the command with SIGINT (roslaunch's own clean shutdown),
then SIGTERM and SIGKILL. Standard library only: no ROS, works before the workspace is sourced.
"""

from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

CALLER = '/x30_watchdog'
STOP_SEC = 25.0  # SIGINT to the command, then SIGTERM; SIGKILL after as long again


class TimeoutTransport(xmlrpc.client.Transport):
    """XML-RPC over HTTP with a timeout: a hung master or node never blocks the watchdog."""

    def __init__(self, timeout: float):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        """Return the HTTP connection with the timeout set."""
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


def proxy(uri: str, timeout: float) -> xmlrpc.client.ServerProxy:
    """Return an XML-RPC proxy whose calls give up after timeout seconds."""
    return xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(timeout))


def notify(message: str):
    """Send a message to systemd's notify socket (nothing outside a service)."""
    path = os.environ.get('NOTIFY_SOCKET', '')
    if not path:
        return
    if path.startswith('@'):
        path = '\0' + path[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)  # a full socket never stalls the checks
            sock.sendto(message.encode(), path)
    except OSError:
        pass  # systemd busy or gone: the next ping tries again


class Checks:
    """The master's run and the critical nodes, as one problem string ('' when all hold)."""

    def __init__(self, master: str, critical: list[str], timeout: float):
        self.master = master
        self.critical = critical
        self.timeout = timeout
        self.run_id = ''

    def master_problem(self) -> tuple[str, bool]:
        """Return (problem, restarted): restarted means the nodes are gone from it at once."""
        try:
            code, _, run_id = proxy(self.master, self.timeout).getParam(CALLER, '/run_id')
        except (OSError, xmlrpc.client.Error) as error:
            return f'master {self.master} away: {error}', False
        if code != 1:
            return f'master {self.master} has no /run_id', False
        if not self.run_id:
            self.run_id = run_id
        elif run_id != self.run_id:
            return f'master restarted (run {self.run_id} -> {run_id})', True
        return '', False

    def node_problem(self, node: str) -> str:
        """Return why a node does not count as up ('' when it does)."""
        try:
            code, _, uri = proxy(self.master, self.timeout).lookupNode(CALLER, node)
            if code != 1:
                return f'{node} not registered'
            proxy(uri, self.timeout).getPid(CALLER)
        except (OSError, xmlrpc.client.Error) as error:
            return f'{node} not answering: {error}'
        return ''

    def problem(self) -> tuple[str, bool]:
        """Return (what is wrong, whether to restart at once)."""
        problem, restarted = self.master_problem()
        if problem:
            return problem, restarted
        missing = [p for p in (self.node_problem(node) for node in self.critical) if p]
        return '; '.join(missing), False


def parse(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    """Return the options and the command after --."""
    if '--' not in argv:
        raise SystemExit('usage: x30_watchdog.py [options] -- command ...')
    split = argv.index('--')
    parser = argparse.ArgumentParser(prog='x30_watchdog', description=__doc__.splitlines()[0])
    parser.add_argument('--critical', action='append', default=[], help='node name (repeat)')
    parser.add_argument('--period', type=float, default=5.0, help='seconds between checks')
    parser.add_argument('--grace', type=float, default=45.0, help='seconds a problem may last')
    parser.add_argument('--startup', type=float, default=240.0, help='seconds to come up')
    parser.add_argument('--timeout', type=float, default=3.0, help='seconds per XML-RPC call')
    parser.add_argument('--master', default=os.environ.get('ROS_MASTER_URI', ''))
    args = parser.parse_args(argv[:split])
    command = argv[split + 1 :]
    if not command or not args.master:
        raise SystemExit('x30_watchdog: a command after -- and ROS_MASTER_URI are needed')
    return args, command


def log(message: str):
    """One line to the journal."""
    print(f'x30_watchdog: {message}', flush=True)


def stop(child: subprocess.Popen) -> int:
    """Stop the command (SIGINT, SIGTERM, SIGKILL) and return its exit code."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if child.poll() is not None:
            break
        child.send_signal(sig)
        try:
            child.wait(STOP_SEC)
        except subprocess.TimeoutExpired:
            log(f'{child.args[0]} still running after {sig.name}')
    return child.wait()


def run(args: argparse.Namespace, command: list[str]) -> int:
    """Supervise the command; return the service's exit code."""
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    child = subprocess.Popen(command)
    checks = Checks(args.master, args.critical, args.timeout)
    start = time.monotonic()
    ready, bad_since, last = False, None, ''
    log(f'started {" ".join(command)}; critical: {", ".join(args.critical) or "none"}')
    while True:
        if stopping:
            log('stopping')
            stop(child)
            return 0
        code = child.poll()
        if code is not None:
            log(f'{command[0]} exited ({code})')
            return code or 1
        problem, at_once = checks.problem()
        now = time.monotonic()
        if problem != last:
            log(problem or 'all up')
            last = problem
        if not problem:
            bad_since = None
            if not ready:
                ready = True
                notify(f'READY=1\nSTATUS=up, {len(args.critical)} critical nodes')
            notify(f'WATCHDOG=1\nSTATUS=up, {len(args.critical)} critical nodes')
        else:
            bad_since = bad_since or now
            limit = args.grace if ready else args.startup
            waited = (now - bad_since) if ready else (now - start)
            if at_once or waited > limit:
                log(f'restart: {problem}')
                notify(f'STATUS=restart: {problem}')
                stop(child)
                return 1
            notify(f'WATCHDOG=1\nSTATUS={"starting" if not ready else "waiting"}: {problem}')
        deadline = time.monotonic() + args.period
        while time.monotonic() < deadline and not stopping and child.poll() is None:
            time.sleep(min(0.2, args.period))


def main(argv: list[str] | None = None) -> int:
    """Run the watchdog."""
    args, command = parse(sys.argv[1:] if argv is None else argv)
    return run(args, command)


if __name__ == '__main__':
    sys.exit(main())
