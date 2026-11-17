"""The private master of the tests: never a port that another process answers on."""

from __future__ import annotations

import os
import threading
from xmlrpc.server import SimpleXMLRPCServer

import pytest

from x30_utils.testing import harness
from x30_utils.testing.harness import pid_at, ros_master

OTHER_PID = 4242


@pytest.fixture
def squatter():
    """A port answering getPid for another process (a parallel test run's master or node)."""
    server = SimpleXMLRPCServer(('localhost', 0), logRequests=False)
    server.register_function(lambda _caller_id: [1, '', OTHER_PID], 'getPid')
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_address[1]
    finally:
        server.shutdown()
        server.server_close()


def test_a_port_taken_meanwhile_is_skipped(squatter, monkeypatch):
    """free_port() said free, another process took it: the next port is tried."""
    ports = iter([squatter])
    free_port = harness.free_port
    monkeypatch.setattr(harness, 'free_port', lambda: next(ports, None) or free_port())
    before = os.environ.get('ROS_MASTER_URI')
    with ros_master() as uri:
        assert not uri.endswith(f':{squatter}')
        assert pid_at(uri) not in (None, OTHER_PID, os.getpid())
        assert os.environ['ROS_MASTER_URI'] == uri
    assert pid_at(uri) is None, 'the master is still running'
    assert os.environ.get('ROS_MASTER_URI') == before


def test_a_fixed_port_taken_is_refused(squatter):
    with pytest.raises(RuntimeError, match='did not start'), ros_master(squatter):
        pass


def test_pid_at_nobody_or_no_ros_api_is_none(squatter):
    assert pid_at(f'http://localhost:{squatter}') == OTHER_PID
    assert pid_at(f'http://localhost:{harness.free_port()}') is None
