"""tools/x30_host.sh and tools/x30_watchdog.py: the two hosts' network, service and watchdog."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import stat
import subprocess
import sys
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools'


def host(*args: str, env: dict | None = None, timeout: float = 60.0):
    return subprocess.run(
        ['bash', str(TOOLS / 'x30_host.sh'), *args],
        capture_output=True,
        text=True,
        env={**os.environ, **(env or {})},
        timeout=timeout,
        check=False,
    )


def test_each_host_joins_the_navigation_hosts_master():
    perception = host('env', 'perception').stdout
    assert 'export ROS_MASTER_URI=http://192.168.1.106:11311' in perception
    assert 'export ROS_IP=192.168.1.105' in perception
    assert 'X30_PYTHON' not in perception
    navigation = host('env', 'navigation').stdout
    assert 'export ROS_IP=192.168.1.106' in navigation
    assert 'unset ROS_HOSTNAME' in navigation and 'PYTHONNOUSERSITE=1' in navigation
    assert host('env', 'motion').returncode == 2


def test_install_writes_the_service_with_its_watchdog_and_keeps_the_settings(tmp_path):
    env = {
        'X30_ETC': str(tmp_path / 'etc'),
        'X30_SYSTEMD_DIR': str(tmp_path),
        'X30_SYSTEMCTL': 'true',
    }
    out = host('install', 'perception', env=env)
    assert out.returncode == 0, out.stdout + out.stderr
    settings = tmp_path / 'etc' / 'perception.env'
    assert stat.S_IMODE(settings.stat().st_mode) == 0o600  # the passwords
    text = settings.read_text()
    assert 'ROS_IP=192.168.1.105' in text and 'HIKMICRO_PASSWORD=' in text
    unit = (tmp_path / 'x30-perception.service').read_text()
    for line in ('Type=notify', 'NotifyAccess=main', 'WatchdogSec=60', 'Restart=on-failure'):
        assert line in unit
    assert f'ExecStart=/bin/bash {TOOLS}/x30_host.sh start perception' in unit
    assert f'EnvironmentFile={settings}' in unit
    settings.write_text('X30_LAUNCH_ARGS="hertzino:=false"\n')
    assert 'kept' in host('install', 'perception', env=env).stdout
    assert settings.read_text() == 'X30_LAUNCH_ARGS="hertzino:=false"\n'
    host('install', 'navigation', env=env)
    navigation = (tmp_path / 'x30-navigation.service').read_text()
    assert 'After=network-online.target jy_roscore.service' in navigation
    # no NeuPAN venv beside this checkout: X30_PYTHON stays a comment
    assert '# X30_PYTHON=' in (tmp_path / 'etc' / 'navigation.env').read_text()


def test_start_runs_the_hosts_launch_under_the_watchdog(tmp_path):
    bin_dir, ws = tmp_path / 'bin', tmp_path / 'ws'
    bin_dir.mkdir()
    (ws / 'devel').mkdir(parents=True)
    (ws / 'devel' / 'setup.bash').write_text('')
    (tmp_path / 'ros.bash').write_text('')
    launched = tmp_path / 'launched'
    fake = bin_dir / 'roslaunch'
    fake.write_text(f'#!/bin/sh\nprintf "%s\\n" "$ROS_IP" "$@" > {launched}\nexit 3\n')
    fake.chmod(0o755)
    env = {
        'PATH': f'{bin_dir}:{os.environ["PATH"]}',
        'X30_ROS_SETUP': str(tmp_path / 'ros.bash'),
        'X30_WS': str(ws),
        'X30_LAUNCH_ARGS': "hertzino:=false hikmicro_host:='10.0.0.9'",
        'X30_SETTLE_SEC': '0',
        'ROS_MASTER_URI': 'http://127.0.0.1:9',
    }
    out = host('start', 'perception', env=env)
    assert out.returncode == 3, out.stdout + out.stderr  # the launch's exit, through the watchdog
    assert launched.read_text().splitlines() == [
        '192.168.1.105',
        '--wait',
        'x30_bringup',
        'perception_host.launch',
        'hertzino:=false',
        'hikmicro_host:=10.0.0.9',
    ]
    nowhere = {
        'X30_LAUNCH_ARGS': "controller:=neupan lidar_pose:='0.3 0 0.1 0 0 0'",
        'X30_PYTHON': str(tmp_path / 'none'),
    }
    out = host('start', 'navigation', env={**env, **nowhere})
    assert out.returncode == 3, out.stdout + out.stderr
    assert 'path follower runs with the system' in out.stdout  # never a missing launch-prefix
    assert launched.read_text().splitlines() == [
        '192.168.1.106',
        '--wait',  # Deep Robotics' master, never one of the launch's own
        'x30_bringup',
        'navigation_host.launch',
        'controller:=neupan',
        'lidar_pose:=0.3 0 0.1 0 0 0',  # one argument
    ]
    assert '/x30/x30_driver' in out.stdout  # the critical nodes, logged by the watchdog


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
