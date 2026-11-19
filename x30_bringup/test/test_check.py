"""tools/x30_host.sh check and x30_check.py: the master, the nodes, the clocks."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools'
SPEC = importlib.util.spec_from_file_location('x30_check', TOOLS / 'x30_check.py')
x30_check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x30_check)


class Publisher:
    """A TCPROS publisher of /x30/odom on 127.0.0.1 whose clock is skew seconds ahead."""

    def __init__(self, skew: float, definition: str = 'Header header\nfloat64 x\n', latched=False):
        self.skew, self.definition, self.latched = skew, definition, latched
        self.listener = socket.create_server(('127.0.0.1', 0))
        self.port = self.listener.getsockname()[1]
        self.stopped = False
        threading.Thread(target=self.serve, daemon=True).start()

    def serve(self):
        while not self.stopped:
            try:
                conn, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self.stream, args=(conn,), daemon=True).start()

    def stream(self, conn: socket.socket):
        reply = {'type': 'nav_msgs/Odometry', 'md5sum': 'x', 'latching': str(int(self.latched))}
        with conn:
            x30_check.read_header(conn)
            conn.sendall(x30_check.tcpros_header({**reply, 'message_definition': self.definition}))
            seq = 0
            while not self.stopped:
                now = time.time() + self.skew
                payload = struct.pack('<3I', seq, int(now), int(now % 1 * 1e9)) + bytes(8)
                try:
                    conn.sendall(struct.pack('<I', len(payload)) + payload)
                except OSError:
                    return
                seq += 1
                time.sleep(0.02)

    def close(self):
        self.stopped = True
        self.listener.close()


class Ros:
    """A master and the driver node publishing /x30/odom (from 127.0.0.1: the other host)."""

    def __init__(self, port: int):
        self.servers = []
        node = self.serve(
            requestTopic=lambda caller, topic, protocols: [1, '', ['TCPROS', '127.0.0.1', port]]
        )
        self.master = self.serve(
            getSystemState=lambda caller: [1, '', [[['/x30/odom', ['/x30/x30_driver']]], [], []]],
            getTopicTypes=lambda caller: [1, '', [['/x30/odom', 'nav_msgs/Odometry']]],
            lookupNode=lambda caller, name: [1, '', node],
        )

    def serve(self, **functions) -> str:
        server = SimpleXMLRPCServer(('127.0.0.1', 0), logRequests=False, allow_none=True)
        for name, function in functions.items():
            server.register_function(function, name)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.servers.append(server)
        return f'http://127.0.0.1:{server.server_address[1]}/'

    def close(self):
        for server in self.servers:
            server.shutdown()
            server.server_close()


@pytest.fixture
def other_host():
    """Start a publisher with a skewed clock and its master; yield the master's URI."""
    started = []

    def start(skew: float, **options) -> str:
        publisher = Publisher(skew, **options)
        ros = Ros(publisher.port)
        started.extend((publisher, ros))
        return ros.master

    yield start
    for thing in started:
        thing.close()
    socket.setdefaulttimeout(None)  # x30_check sets it for its run


def clock_row(capsys, master: str) -> str:
    assert x30_check.main(['--master', master, '--other', '127.0.0.1']) == 0
    rows = capsys.readouterr().out.splitlines()
    assert rows[0].startswith(f'master   ok    {master}'), rows
    assert rows[1] == 'nodes    ok    1 of 1 reachable from here', rows
    return rows[2]


def test_clocks_in_step(other_host, capsys):
    row = clock_row(capsys, other_host(0.0))
    assert row.startswith('clock    ok    in step with 127.0.0.1 (its /x30/odom: +'), row


def test_the_other_clock_ahead_or_behind(other_host, capsys):
    ahead = clock_row(capsys, other_host(3.0))
    assert re.match(r'clock    WARN  127\.0\.0\.1 is (2\.99|3\.00) s ahead of this host', ahead)
    behind = clock_row(capsys, other_host(-2.0))
    assert re.match(r'clock    WARN  127\.0\.0\.1 is (2\.00|2\.01) s behind this host', behind)


def test_no_clock_from_a_latched_or_unstamped_topic(other_host, capsys):
    latched = clock_row(capsys, other_host(5.0, latched=True))
    assert latched.startswith('clock    -     nothing stamped from 127.0.0.1'), latched
    plain = clock_row(capsys, other_host(5.0, definition='float64 x\n'))
    assert plain.startswith('clock    -     nothing stamped'), plain


def test_check_on_a_laptop_with_a_loopback_ros_ip_and_no_master():
    env = {**os.environ, 'ROS_IP': '127.0.0.1', 'ROS_MASTER_URI': 'http://127.0.0.1:9'}
    out = subprocess.run(
        ['bash', str(TOOLS / 'x30_host.sh'), 'check'],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=False,
    )
    assert out.returncode == 1, out.stdout + out.stderr
    assert 'ros_ip   MISS  ROS_IP 127.0.0.1: loopback' in out.stdout
    assert 'master   MISS  http://127.0.0.1:9' in out.stdout
