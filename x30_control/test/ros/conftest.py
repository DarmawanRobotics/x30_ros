"""Test rig: the control mux in this process, with publishers, service proxies and collectors."""

from __future__ import annotations

import itertools
import math
import threading
import time

from geometry_msgs.msg import Twist
import pytest
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from x30_control.control_mux_node import ControlMuxNode
from x30_msgs.msg import ControlStatus, HealthStatus, SectorScan
from x30_msgs.srv import SetMode
from x30_utils.testing.fixtures import ros  # noqa: F401
from x30_utils.testing.harness import Collector, connected, delete_params

N = 72
_ids = itertools.count()


def twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
    return msg


class Rig:
    """Mux under test, a scan producer (10 Hz) and recorders."""

    def __init__(self, **params):
        self.ns = f'/control_mux_test_{next(_ids)}'
        self.mux = ControlMuxNode(param_ns=self.ns, overrides=params)
        self.out = Collector(Twist, 'cmd_vel', 1000)
        self.status = Collector(ControlStatus, 'control/status', 1000)
        self.nav = rospy.Publisher('control/nav_cmd_vel', Twist, queue_size=10)
        self.remote = {
            s: rospy.Publisher(f'control/remote/{s}/cmd_vel', Twist, queue_size=10)
            for s in ('handheld', 'dashboard')
        }
        self.request = rospy.Publisher('control/estop_request', String, queue_size=10)
        self.hw = rospy.Publisher('control/hw_estop', Bool, queue_size=10)
        self.health = rospy.Publisher('health/status', HealthStatus, queue_size=10)
        self.scan_pub = rospy.Publisher('perception/safety_scan', SectorScan, queue_size=10)
        self.ranges = [4.0] * N
        self.frame = 'base_link'
        self.scanning = True
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        while not self._stop.wait(0.1):
            if self.scanning:
                self.publish_scan()

    def publish_scan(self):
        msg = SectorScan()
        msg.header.frame_id = self.frame
        msg.header.stamp = rospy.Time.now()
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / N
        msg.range_max = 4.0
        msg.ranges = [float(r) for r in self.ranges]
        self.scan_pub.publish(msg)

    def call(self, name: str, srv=Trigger, **request):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, srv)(**request)

    def engage(self):
        return self.call('control/estop/engage')

    def release(self):
        return self.call('control/estop/release')

    def mode(self, mode: str, source: str = ''):
        """Switch the mode; on success wait for its status (a status before it may still come)."""
        reply = self.call('control/set_mode', SetMode, mode=mode, source=source)
        if reply.success:
            remote = mode == 'remote'
            want = ControlStatus.MODE_REMOTE if remote else ControlStatus.MODE_AUTO

            def switched(s):
                return s.mode == want and (not remote or not source or s.remote_source == source)

            assert self.wait_status(switched)
        return reply

    def stream(self, pub, msg: Twist, seconds: float, hz: float = 20.0):
        """Publish msg at hz for seconds."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pub.publish(msg)
            time.sleep(1.0 / hz)

    def last_status(self) -> ControlStatus:
        assert self.status.wait(1, 3.0)
        return self.status.last

    def wait_status(self, cond, timeout: float = 3.0) -> bool:
        return self.status.wait_until(cond, timeout)

    def outputs_since(self, start: int) -> list:
        return self.out.messages[start:]

    def ready(self, timeout: float = 5.0):
        """Wait for every connection, the first scans and navigation re-armed."""
        pubs = (self.scan_pub, self.nav, *self.remote.values(), self.request, self.hw)
        assert connected(*pubs, self.out, self.status, timeout=timeout)
        assert self.wait_status(lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR)

    def close(self):
        self._stop.set()
        self._thread.join(1.0)
        self.mux.destroy_node()
        for handle in (self.nav, *self.remote.values(), self.request, self.hw, self.health):
            handle.unregister()
        self.scan_pub.unregister()
        self.out.close()
        self.status.close()
        delete_params(self.ns)


@pytest.fixture
def make_rig(ros):  # noqa: F811
    """Return a factory for started rigs (parameters as keyword arguments); tear down after."""
    rigs = []

    def factory(ready: bool = True, **params) -> Rig:
        rig = Rig(**params)
        rigs.append(rig)
        if ready:
            rig.ready()
        return rig

    yield factory
    for rig in rigs:
        rig.close()
