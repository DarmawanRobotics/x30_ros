"""The mux with /use_sim_time and a stopped /clock: its watchdog and hold still run."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time

from conftest import N, twist
from geometry_msgs.msg import Twist
import pytest
import rospy
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, SectorScan
from x30_msgs.srv import SetMode
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for


def moving(msgs) -> list:
    return [m for m in msgs if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6]


@pytest.fixture
def sim_mux(ros):
    """control_mux_node in its own process with simulated time and no /clock."""
    rospy.set_param('/use_sim_time', True)
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    proc = subprocess.Popen(
        [sys.executable, '-m', 'x30_control.control_mux_node', '__name:=control_mux_sim'],
        env=env,
    )
    try:
        yield proc
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(15.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        delete_params('/use_sim_time')
        delete_params('/control_mux_sim')


def scan(stamp) -> SectorScan:
    msg = SectorScan()
    msg.header.frame_id, msg.header.stamp = 'base_link', stamp
    msg.angle_min, msg.angle_increment, msg.range_max = -math.pi, 2 * math.pi / N, 4.0
    msg.ranges = [4.0] * N
    return msg


def test_watchdog_and_hold_run_on_steady_time_with_sim_time(sim_mux):
    out = Collector(Twist, 'cmd_vel', 1000)
    status = Collector(ControlStatus, 'control/status', 1000)
    handheld = rospy.Publisher('control/remote/handheld/cmd_vel', Twist, queue_size=10)
    scans = rospy.Publisher('perception/safety_scan', SectorScan, queue_size=10)
    try:
        assert connected(out, status, handheld, scans, timeout=15.0)
        rospy.wait_for_service('control/set_mode', 10.0)
        mode = rospy.ServiceProxy('control/set_mode', SetMode)
        assert mode(mode='remote', source='handheld').success
        for _ in range(5):  # stamps from the future for sim time 0: STALE
            scans.publish(scan(rospy.Time.now()))
            time.sleep(0.1)
        assert status.wait_until(lambda s: s.field == ControlStatus.FIELD_STALE)
        assert wait_for(lambda: _rearmed(handheld, status), 5.0)
        start = len(out.messages)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            handheld.publish(twist(0.5))
            time.sleep(0.05)
        time.sleep(0.05)
        drove = out.messages[start:]
        assert drove and drove[-1].linear.x == pytest.approx(0.15, abs=1e-3)  # creeps
        start = len(out.messages)
        time.sleep(0.6)
        after = out.messages[start:]
        zeros = [i for i, msg in enumerate(after) if not moving([msg])]
        assert zeros == [len(after) - 1]  # one zero after remote_timeout_sec, nothing after
        rospy.wait_for_service('control/estop/engage', 5.0)
        assert rospy.ServiceProxy('control/estop/engage', Trigger)().success
        start = len(out.messages)
        time.sleep(0.5)
        assert len(out.messages[start:]) >= 4  # hold stream at estop_hold_hz
    finally:
        out.close()
        status.close()
        handheld.unregister()
        scans.unregister()


def _rearmed(pub, status) -> bool:
    pub.publish(twist())
    time.sleep(0.05)
    return status.last is not None and status.last.armed
