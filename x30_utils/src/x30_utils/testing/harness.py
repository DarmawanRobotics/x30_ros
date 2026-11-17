"""ROS 1 test helpers: a private master, one test node per process, collectors and waits."""

from __future__ import annotations

import contextlib
import os
import socket
import subprocess
import threading
import time
import xmlrpc.client

import rospy

MASTER_START_SEC = 15.0
MASTER_ATTEMPTS = 5  # ports to try: another process may take a free port before rosmaster binds it
PROBE_SEC = 1.0  # one XML-RPC probe of a port


def wait_for(predicate, timeout: float = 5.0, period: float = 0.01) -> bool:
    """Poll predicate() until it is true; return False after timeout."""
    deadline = time.monotonic() + timeout
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(period)


def free_port() -> int:
    """Return a TCP port nobody listens on right now."""
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('localhost', 0))
        return s.getsockname()[1]


class _ProbeTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = PROBE_SEC  # whatever listens there may never answer
        return connection


def pid_at(uri: str) -> int | None:
    """Return the pid of the master or node answering at uri, None when none does."""
    proxy = xmlrpc.client.ServerProxy(uri, transport=_ProbeTransport())
    try:
        code, _message, pid = proxy.getPid('/x30_test_probe')
    except Exception:  # nobody listens, or not the ROS XML-RPC API
        return None
    return pid if code == 1 else None


def _stop(process: subprocess.Popen):
    process.terminate()
    try:
        process.wait(5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _start_master(port: int) -> subprocess.Popen | None:
    """Start rosmaster on port; None when another process answers there."""
    uri = f'http://localhost:{port}'
    master = subprocess.Popen(
        ['rosmaster', '--core', '-p', str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # rosmaster waits forever when its port is taken, and whoever has it answers for it: the
    # master is ours only when the pid answering is its own
    wait_for(lambda: master.poll() is not None or pid_at(uri) is not None, MASTER_START_SEC, 0.1)
    if master.poll() is None and pid_at(uri) == master.pid:
        return master
    _stop(master)
    return None


@contextlib.contextmanager
def ros_master(port: int | None = None):
    """Run a private rosmaster for the tests and point ROS_MASTER_URI at it."""
    # a free port is only free now: parallel test runs (catkin_make -j) bind ports meanwhile
    for _ in range(1 if port else MASTER_ATTEMPTS):
        chosen = port or free_port()
        master = _start_master(chosen)
        if master is not None:
            break
    else:
        raise RuntimeError(f'rosmaster did not start (last port {chosen})')
    uri = f'http://localhost:{chosen}'
    previous = os.environ.get('ROS_MASTER_URI')
    os.environ['ROS_MASTER_URI'] = uri
    try:
        yield uri
    finally:
        _stop(master)
        if previous is None:
            os.environ.pop('ROS_MASTER_URI', None)
        else:
            os.environ['ROS_MASTER_URI'] = previous


def init_test_node(name: str = 'x30_test'):
    """Init the one rospy node of the test process (no signal handlers)."""
    if not rospy.core.is_initialized():
        rospy.init_node(name, anonymous=True, disable_signals=True)


def connected(*handles, timeout: float = 5.0) -> bool:
    """Wait until every publisher and subscriber has at least one connection."""
    return wait_for(lambda: all(h.get_num_connections() > 0 for h in handles), timeout)


class Collector:
    """Subscribe and keep every message of a topic (thread-safe)."""

    def __init__(self, msg_type, topic: str, queue_size: int = 100):
        self._lock = threading.Lock()
        self._messages: list = []
        self.sub = rospy.Subscriber(topic, msg_type, self._on_message, queue_size=queue_size)

    def _on_message(self, msg):
        with self._lock:
            self._messages.append(msg)

    @property
    def messages(self) -> list:
        """A copy of what arrived so far."""
        with self._lock:
            return list(self._messages)

    @property
    def last(self):
        """The newest message, or None."""
        with self._lock:
            return self._messages[-1] if self._messages else None

    def clear(self):
        """Forget what arrived."""
        with self._lock:
            self._messages.clear()

    def wait(self, count: int = 1, timeout: float = 5.0) -> bool:
        """Wait until at least count messages arrived."""
        return wait_for(lambda: len(self.messages) >= count, timeout)

    def wait_until(self, predicate, timeout: float = 5.0) -> bool:
        """Wait until the newest message satisfies predicate."""
        return wait_for(lambda: self.last is not None and predicate(self.last), timeout)

    def get_num_connections(self) -> int:
        """Return the publishers connected to the subscriber."""
        return self.sub.get_num_connections()

    def close(self):
        """Unsubscribe."""
        self.sub.unregister()


def delete_params(namespace: str):
    """Delete a parameter namespace if it exists."""
    with contextlib.suppress(KeyError, rospy.ROSException):
        if rospy.has_param(namespace):
            rospy.delete_param(namespace)
