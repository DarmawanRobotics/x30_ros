"""In-process test of odometry_monitor_node: typed status, restart event, turn in place, trip."""

from __future__ import annotations

import itertools
import json
import math

from nav_msgs.msg import Odometry
import pytest
import rospy
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from x30_msgs.msg import OdometryStatus, TripStatus
from x30_odometry.odometry_monitor_node import OdometryMonitorNode
from x30_utils.config import ConfigError
from x30_utils.ros.time import to_stamp
from x30_utils.testing.harness import Collector, connected, wait_for

NAMES = itertools.count()


class Rig:
    """The monitor, a publisher of /Odometry and collectors of its outputs, in this process."""

    def __init__(self, trip_file: str = ''):
        n = next(NAMES)
        self.odom_topic = f'/monitor_test_{n}/Odometry'
        self.node = OdometryMonitorNode(
            param_ns=f'/monitor_test_{n}/node',
            overrides={
                'odom_topic': self.odom_topic,
                'imu_topic': '',
                'leg_odom_topic': f'/monitor_test_{n}/leg_odom',
                'robot.limits.max_vx': 1.0,
                'trip.file': trip_file,  # never the real ~/.x30
            },
        )
        self.status = Collector(OdometryStatus, 'odometry/status')
        self.trip = Collector(TripStatus, 'odometry/trip')
        self.restarts = Collector(Empty, 'odometry/restarted')
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.t = 1000.0
        assert connected(self.pub, self.restarts, self.status)

    def odom(self, x: float, yaw: float = 0.0, dt: float = 0.1):
        """Publish the estimator pose dt s after the last one (stamp), then wait as long."""
        self.t += dt
        msg = Odometry()
        msg.header.stamp = to_stamp(self.t)
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)
        rospy.sleep(0.1)

    def close(self):
        self.node.run_shutdown_hooks()
        self.node.destroy_node()
        for handle in (self.status, self.trip, self.restarts):
            handle.close()
        self.pub.unregister()


def test_status_is_typed_and_restart_needs_gap_and_origin(ros):
    rig = Rig()
    try:
        assert rig.status.wait_until(lambda m: m.state == OdometryStatus.INITIALIZING)
        for k in range(10):
            rig.odom(x=0.05 * k)
        assert rig.status.wait_until(lambda m: m.state == OdometryStatus.OK)
        last = rig.status.last
        assert last.message == '' and 0.3 < last.speed < 0.7
        rospy.sleep(1.0)  # the estimator is down (respawn): stale
        assert rig.status.last.state == OdometryStatus.LOST
        rig.odom(x=0.0, dt=4.0)  # back at the origin after the gap
        assert rig.restarts.wait(1, timeout=2.0)
        assert rig.status.wait_until(lambda m: m.message.startswith('restarted'))
        assert rig.status.last.state == OdometryStatus.LOST
    finally:
        rig.close()


def test_turn_in_place_at_the_start_is_no_restart(ros):
    """Yaw 90 -> 0 deg at 2 cm from the start."""
    rig = Rig()
    try:
        for k in range(19):
            rig.odom(x=0.02, yaw=math.radians(90.0 - 5.0 * k))
        rospy.sleep(0.3)
        assert not rig.restarts.messages
        assert rig.status.last.state == OdometryStatus.OK
    finally:
        rig.close()


def test_invalid_parameters_are_refused(ros):
    with pytest.raises(ConfigError, match=r'restart\.gap_sec'):
        OdometryMonitorNode(
            param_ns='/monitor_test_invalid/node',
            overrides={'imu_topic': '', 'restart.gap_sec': 0.2},  # must stay above stale_sec
        )


def test_live_parameters_reach_the_monitor_or_are_refused(ros):
    rig = Rig()
    ns = rig.node.param_namespace
    try:
        rospy.set_param(f'{ns}/restart/gap_sec', 0.2)  # must stay above stale_sec
        rospy.set_param(f'{ns}/hold_sec', 1.0)
        rospy.sleep(2.5)  # two polls: the values on the server are refused as a whole
        assert rig.node.monitor.config.hold_sec == 5.0
        rospy.set_param(f'{ns}/restart/gap_sec', 3.0)
        assert wait_for(lambda: rig.node.monitor.config.restart.gap_sec == 3.0, timeout=5.0)
        assert rig.node.monitor.config.hold_sec == 1.0
    finally:
        rig.close()
        rospy.delete_param(ns)


def call_reset():
    rospy.wait_for_service('odometry/trip_reset', 5.0)
    return rospy.ServiceProxy('odometry/trip_reset', Trigger)()


def test_trip_is_counted_kept_across_a_restart_and_reset(ros, tmp_path):
    """odometry/trip from the poses; the totals survive a restart; odometry/trip_reset zeroes."""
    file = str(tmp_path / 'trip.json')
    rig = Rig(trip_file=file)
    try:
        for k in range(1, 31):  # 3 s at 0.5 m/s
            rig.odom(x=0.05 * k)
        rospy.sleep(0.6)
        last = rig.trip.last
        assert 1.3 < last.distance < 1.6
        assert 0.4 < last.speed < 0.6 and 0.4 < last.max_speed < 0.6
        assert 0.4 < last.average < 0.6 and 2.5 < last.moving_sec < 3.2
        assert last.since.secs > 1_600_000_000  # wall clock of the reset (the start here)
    finally:
        rig.close()
    with open(file, encoding='utf-8') as f:
        kept = json.load(f)
    assert 1.3 < kept['distance_m'] < 1.6

    rig = Rig(trip_file=file)  # the robot restarts
    try:
        assert rig.trip.wait_until(lambda m: abs(m.distance - kept['distance_m']) < 1e-3)
        response = call_reset()
        assert response.success and 'was 1.' in response.message
        assert rig.trip.wait_until(lambda m: m.distance == 0.0 and m.max_speed == 0.0)
    finally:
        rig.close()
    with open(file, encoding='utf-8') as f:
        assert json.load(f)['distance_m'] == 0.0
