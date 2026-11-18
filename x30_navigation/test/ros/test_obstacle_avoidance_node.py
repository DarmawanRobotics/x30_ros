"""obstacle_avoidance_node in-process: blind scans are an emergency, policies switch live."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math

from geometry_msgs.msg import Twist, Vector3
from ros_rig import set_param
import rospy

from x30_msgs.msg import AvoidanceCommand, SectorScan
from x30_navigation.core.avoidance import POLICIES, AvoidanceOut, AvoidancePolicy
from x30_navigation.obstacle_avoidance_node import ObstacleAvoidanceNode
from x30_utils.config import param
from x30_utils.testing.harness import Collector, delete_params, wait_for

NAMES = itertools.count()


def scan(value: float) -> SectorScan:
    msg = SectorScan(angle_min=-math.pi / 2, angle_increment=math.pi / 32, range_max=8.0)
    msg.header.frame_id = 'base_link'
    msg.ranges = [value] * 32
    return msg


def make(**overrides) -> ObstacleAvoidanceNode:
    return ObstacleAvoidanceNode(param_ns=f'/avoidance_test_{next(NAMES)}', overrides=overrides)


def test_blind_scan_is_an_emergency_and_policies_switch_live(ros):
    """All sectors BLOCKED (0) or NaN: emergency, speed 0."""
    node = make()
    out = Collector(AvoidanceCommand, 'navigation/avoidance')
    pub = rospy.Publisher('perception/sector_scan', SectorScan, queue_size=10)
    ns = node.param_namespace
    try:
        for value, emergency in ((8.0, False), (0.0, True), (math.nan, True), (8.0, False)):
            sent = scan(value)

            def seen(m=sent, e=emergency):
                pub.publish(m)
                return out.last is not None and out.last.emergency is e

            assert wait_for(seen, 3.0), value
        assert set_param(node, 'policy', 'nope', 'rejected')
        assert set_param(node, 'policy', 'follow_gap')
        assert type(node.policy).__name__ == 'FollowGapAvoidance'
    finally:
        out.close()
        pub.unregister()
        node.destroy_node()
        delete_params(ns)


def behind(value: float) -> SectorScan:
    """A 360 deg front scan with an obstacle at value m straight behind the robot."""
    msg = SectorScan(angle_min=-math.pi, angle_increment=math.pi / 36, range_max=8.0)
    msg.header.frame_id = 'base_link'
    angles = [-math.pi + (i + 0.5) * math.pi / 36 for i in range(72)]
    msg.ranges = [value if abs(a) > 2.9 else 8.0 for a in angles]
    return msg


def test_reversing_looks_behind_on_a_360_scan(ros):
    """360 deg avoidance: an obstacle behind matters only while navigation reverses."""
    node = make()
    out = Collector(AvoidanceCommand, 'navigation/avoidance')
    pub = rospy.Publisher('perception/sector_scan', SectorScan, queue_size=10)
    cmd = rospy.Publisher('control/nav_cmd_vel', Twist, queue_size=10)
    try:
        for vx, emergency in ((0.3, False), (-0.1, True), (0.3, False)):

            def seen(v=vx, e=emergency):
                cmd.publish(Twist(linear=Vector3(x=v)))
                pub.publish(behind(0.5))
                return out.last is not None and out.last.emergency is e

            assert wait_for(seen, 3.0), vx
    finally:
        out.close()
        pub.unregister()
        cmd.unregister()
        node.destroy_node()
        delete_params(node.param_namespace)


@dataclass(frozen=True)
class DemoConfig:
    gain: float = param(0.5, min=0.0, max=1.0)


class DemoAvoidance(AvoidancePolicy):
    Config = DemoConfig

    def compute(self, angles, ranges, scan):
        return AvoidanceOut(velocity_scale=self.config.gain)


def test_plugin_policy_gets_its_own_parameters(ros, monkeypatch):
    """A registered plugin policy is configured through <name>/* like the built-in ones."""
    POLICIES.names()  # discover the installed policies first
    monkeypatch.setitem(POLICIES._classes, 'demo_plugin', DemoAvoidance)  # removed after the test
    ns = f'/avoidance_test_{next(NAMES)}'
    rospy.set_param(f'{ns}/demo_plugin/gain', 0.3)
    node = ObstacleAvoidanceNode(param_ns=ns, overrides={'policy': 'demo_plugin'})
    try:
        assert node.policy.config.gain == 0.3
        assert set_param(node, 'demo_plugin/gain', 2.0, 'rejected')  # above its maximum
        assert set_param(node, 'demo_plugin/gain', 0.8)
        assert node.policy.config.gain == 0.8
    finally:
        node.destroy_node()
        delete_params(ns)
