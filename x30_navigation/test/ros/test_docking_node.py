"""docking_node in-process: a kinematic robot, a tag on TF, sit / charge hooks, a battery."""

from __future__ import annotations

import itertools
import math
from types import SimpleNamespace

from geometry_msgs.msg import Twist
import pytest
from ros_rig import call
import rospy
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, DockingStatus
from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.config.follower import SafetyConfig
from x30_navigation.docking_node import DockingNode
from x30_navigation.utils.control_check import FIELD_PARAMETERS
from x30_utils.math import Pose2D, make_transform, quat_from_rpy
from x30_utils.robot import RobotProfile
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, delete_params, wait_for

S = DockingStatus
TAG = Pose2D(2.5, 0.2, math.pi)  # the tag in odom, its normal toward -x
NAMES = itertools.count()


class DockWorld:
    """Drives a pose with control/nav_cmd_vel; publishes odom -> base_link and the tag."""

    def __init__(self, ns: str):
        self.node = X30Node('dock_world', param_ns=f'{ns}/world')
        self.pose, self.cmd, self.charging, self.calls = Pose2D(1.0, 0.0, 0.1), Twist(), False, []
        self.battery = self.node.create_publisher(BatteryState, 'battery', 10)
        self.node.create_subscription(Twist, 'control/nav_cmd_vel', self.on_cmd, 10)
        for name in ('sit', 'charge', 'charge_stop', 'stand'):
            self.node.create_service(Trigger, f'{ns}/robot/{name}', self.hook(name))
        self.node.create_service(Trigger, 'navigation/stop', self.hook('navigation_stop'))
        self.node.every(0.05, self.tick, steady=True)

    def hook(self, name):
        def answer(_request, response):
            self.calls.append(name)
            self.charging = {'charge': True, 'charge_stop': False}.get(name, self.charging)
            response.success, response.message = True, f'{name} ok'
            return response

        return answer

    def on_cmd(self, msg: Twist):
        self.cmd = msg

    def tick(self):
        c, dt = self.cmd, 0.05
        self.pose = self.pose @ Pose2D(c.linear.x * dt, c.linear.y * dt, c.angular.z * dt)
        stamp = self.node.stamp()
        p = self.pose
        base = make_transform((p.x, p.y, 0.0), quat_from_rpy(0.0, 0.0, p.yaw))
        self.node.tf_out.send(base, 'odom', 'base_link', stamp)
        local = p.between(TAG)
        if math.hypot(local.x, local.y) > 0.4:  # a camera sees it until very close
            # tag frame: z along the normal toward the viewer, as the simulator's plates
            yaw = local.yaw - math.pi / 2.0
            tag = make_transform((local.x, local.y, 0.4), quat_from_rpy(-math.pi / 2.0, 0.0, yaw))
            self.node.tf_out.send(tag, 'base_link', 'charging_dock', stamp)
        status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging else 2
        self.battery.publish(BatteryState(percentage=0.5, power_supply_status=status))

    def close(self):
        self.node.destroy_node()


@pytest.fixture
def rig(ros):
    ns = f'/docking_test_{next(NAMES)}'
    docking = DockingNode(
        param_ns=f'{ns}/docking',
        overrides={
            'control_node': '',
            'charge.confirm_timeout_sec': 30.0,
            'hooks.sit': [f'{ns}/robot/sit'],
            'hooks.charge': [f'{ns}/robot/charge'],
            'hooks.stop_charge': [f'{ns}/robot/charge_stop'],
            'hooks.stand': [f'{ns}/robot/stand'],
            'hooks.sit_settle_sec': 0.2,
            'hooks.stand_settle_sec': 0.2,
            'settle.time_sec': 0.3,
        },
    )
    world = DockWorld(ns)
    status = Collector(DockingStatus, 'docking/status')
    yield docking, world, status
    status.close()
    docking.run_shutdown_hooks()
    docking.destroy_node()
    world.close()
    delete_params(ns)


def test_dock_sit_charge_and_undock(rig):
    """Dock onto the tag, call the hooks in order, confirm charging, undock again."""
    docking, world, status = rig
    assert wait_for(lambda: docking.pose_input() is not None, 5.0)
    assert call(Trigger, 'docking/dock').success
    assert wait_for(lambda: status.last is not None and status.last.state == S.DOCKED, 60.0), (
        status.last
    )
    assert status.last.charging and status.last.attempt == 1
    assert world.calls == ['navigation_stop', 'sit', 'charge']
    goal_x = TAG.x - docking.config.goal.distance
    assert abs(world.pose.x - goal_x) < 0.05 and abs(world.pose.y - TAG.y) < 0.04
    assert not call(Trigger, 'docking/dock').success  # undock first
    assert call(Trigger, 'docking/undock').success
    assert wait_for(lambda: status.last.state == S.IDLE, 30.0), status.last
    assert world.calls[-2:] == ['charge_stop', 'stand']
    assert world.pose.x < goal_x - docking.config.undock.distance + 0.05


def test_estop_fails_docking_and_zero_is_published(rig):
    docking, _world, status = rig
    cmds = Collector(Twist, 'control/nav_cmd_vel')
    control = rospy.Publisher('control/status', ControlStatus, queue_size=1, latch=True)
    try:
        assert wait_for(lambda: docking.pose_input() is not None, 5.0)
        assert call(Trigger, 'docking/dock').success
        approaching = lambda: status.last is not None and status.last.state == S.APPROACHING  # noqa: E731
        assert wait_for(approaching, 30.0)
        control.publish(ControlStatus(estop=True, armed=False))
        assert wait_for(lambda: status.last.state == S.FAILED, 3.0)
        assert 'e-stop' in status.last.message
        last = cmds.last
        assert last.linear.x == last.linear.y == last.angular.z == 0.0
        assert call(Trigger, 'docking/cancel').success
        assert wait_for(lambda: status.last.state == S.IDLE, 2.0)
    finally:
        cmds.close()
        control.unregister()


def test_both_field_checks_take_every_value_of_the_field():
    """ControlFieldCheck hands a check the mux's FIELD_PARAMETERS: the docking node's too."""
    values = (0.10, 0.5, 0.15, 0.10)  # the mux field's defaults, in FIELD_PARAMETERS' order
    assert len(values) == len(FIELD_PARAMETERS)
    assert SafetyConfig().check_control(*values) == []
    docking = SimpleNamespace(config=DockingConfig(), robot=SimpleNamespace(value=RobotProfile()))
    assert DockingNode.check_field(docking, *values) == []
