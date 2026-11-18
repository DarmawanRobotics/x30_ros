"""tools/x30_watchdog.py: a host's roslaunch under systemd's watchdog."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools'


class FakeRos:
    """A master (getParam /run_id, lookupNode) and one node (getPid) over XML-RPC."""

    def __init__(self):
        self.run_id = 'run-1'
        self.nodes: dict[str, str] = {}
        self.node = self.serve({'getPid': lambda caller: [1, '', 42]})
        self.master = self.serve(
            {
                'getParam': lambda caller, key: [1, '', self.run_id],
                'lookupNode': lambda caller, name: (
                    [1, '', self.nodes[name]] if name in self.nodes else [-1, 'unknown', '']
                ),
            }
        )

    def serve(self, functions: dict) -> str:
        server = SimpleXMLRPCServer(('127.0.0.1', 0), logRequests=False, allow_none=True)
        for name, function in functions.items():
            server.register_function(function, name)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.servers = [*getattr(self, 'servers', []), server]
        return f'http://127.0.0.1:{server.server_address[1]}/'

    def close(self):
        for server in self.servers:
            server.shutdown()
            server.server_close()


@pytest.fixture
def ros():
    fake = FakeRos()
    yield fake
    fake.close()


@pytest.fixture
def systemd(tmp_path):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    path = tmp_path / 'notify'
    sock.bind(str(path))
    sock.settimeout(0.1)
    messages = []

    def received() -> list[str]:
        while True:
            try:
                messages.append(sock.recv(4096).decode())
            except (socket.timeout, BlockingIOError):
                return messages

    yield str(path), received
    sock.close()


def watchdog(ros, notify: str, *options: str, command=('sleep', '60')):
    return subprocess.Popen(
        [sys.executable, str(TOOLS / 'x30_watchdog.py'), '--period', '0.2', '--grace', '1',
         '--startup', '3', '--timeout', '1', *options, '--', *command],
        env={**os.environ, 'ROS_MASTER_URI': ros.master, 'NOTIFY_SOCKET': notify},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )  # fmt: skip


def until(predicate, timeout: float = 5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def test_the_watchdog_is_ready_once_the_nodes_answer_and_pings_while_they_do(ros, systemd):
    path, received = systemd
    ros.nodes['/x30/x30_driver'] = ros.node
    process = watchdog(ros, path, '--critical', '/x30/x30_driver')
    assert until(lambda: any(m.startswith('READY=1') for m in received()))
    assert until(lambda: sum('WATCHDOG=1' in m for m in received()) >= 3)
    process.send_signal(signal.SIGTERM)
    assert process.wait(10) == 0  # stopped on purpose: no restart
    assert 'stopping' in process.stdout.read()


def test_a_master_restart_restarts_the_service_at_once(ros, systemd):
    process = watchdog(ros, systemd[0])
    assert until(lambda: any(m.startswith('READY=1') for m in systemd[1]()))
    ros.run_id = 'run-2'
    assert process.wait(5) == 1
    assert 'master restarted (run run-1 -> run-2)' in process.stdout.read()


def test_a_critical_node_away_longer_than_the_grace_restarts_the_service(ros, systemd):
    ros.nodes['/control_mux_node'] = ros.node
    process = watchdog(ros, systemd[0], '--critical', '/control_mux_node')
    assert until(lambda: any(m.startswith('READY=1') for m in systemd[1]()))
    del ros.nodes['/control_mux_node']
    start = time.monotonic()
    assert process.wait(10) == 1
    assert time.monotonic() - start >= 0.9  # the grace: a respawn would have been in time
    assert '/control_mux_node not registered' in process.stdout.read()


def test_a_start_that_never_comes_up_and_a_launch_that_exits_end_the_service(ros, systemd):
    process = watchdog(ros, systemd[0], '--critical', '/never')
    assert process.wait(10) == 1
    assert not any(m.startswith('READY=1') for m in systemd[1]())
    failed = watchdog(ros, systemd[0], command=('sh', '-c', 'exit 7'))
    assert failed.wait(10) == 7
