"""Reactive avoidance on the route: the chosen heading is driven, the defaults let it pass."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import ROBOT, Rig, S, front_scan, person
from sim import route_of, straight_route

from x30_navigation.core.avoidance import POLICIES, AvoidanceOut, AvoidanceShared
from x30_navigation.core.config.avoidance import AvoidanceConfig
from x30_navigation.core.config.follower import FollowerConfig, SafetyConfig
from x30_navigation.core.config.shift import DetourConfig
from x30_navigation.core.controllers import CONTROLLERS, Bias, ControllerLimits, TrackRequest
from x30_navigation.core.separation import protective_distance
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile

REACTIVE = FollowerConfig(require_localization=False, avoidance_mode='reactive')


def holonomic():
    return CONTROLLERS.create('holonomic', None, ControllerLimits(0.8, 0.4, 1.0))


@pytest.mark.parametrize('offset', [-0.6, 0.4])
def test_strafing_controller_drives_the_avoidance_heading(offset):
    """E4: the strafing controller drives the policy's heading, not back to the carrot."""
    route = route_of(straight_route(0.0, 20.0))
    request = TrackRequest(route, 2.0, Pose2D(2.0, -0.4, 0.0), 0.4, 0.05, bias=Bias(offset))
    command = holonomic().update(request)
    assert math.atan2(command.vy, command.vx) == pytest.approx(offset, abs=1e-9)
    assert math.hypot(command.vx, command.vy) == pytest.approx(0.4, abs=0.05)


def test_held_pull_back_goes_on_parallel_to_the_route():
    """A pull back held beside a person goes on parallel until the way is clear."""
    rig = Rig(route_of(straight_route(0.0, 10.0)), REACTIVE, pose=Pose2D(3.4, 0.55, 0.0))
    rig.sectors = 72
    rig.obstacles = person(4.0, -0.15, n=72)
    assert rig.follower.start(rig.pose)[0]
    rig.run(30.0, until=lambda d: rig.pose.x > 5.5)
    assert rig.pose.x > 5.5 and abs(rig.pose.y) < 0.6
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def drive_past(y: float, shared: AvoidanceShared, seconds: float = 40.0, config=REACTIVE) -> Rig:
    """Close the loop: VFH on the front scan, the follower on the 360 deg sector scan."""
    rig = Rig(route_of(straight_route(0.0, 10.0)), config)
    rig.sectors = 72
    rig.obstacles = person(4.0, y, n=72)
    config = AvoidanceConfig()
    policy = POLICIES.create(config.policy, shared, config.policy_config(), robot=ROBOT)
    assert rig.follower.start(rig.pose)[0]
    for _ in range(int(seconds / rig.dt)):
        rig.avoidance = policy.update(front_scan(rig.pose.inverse().apply(rig.obstacles)))
        decision = rig.tick()
        if rig.pose.x > 6.0 or decision.report.state is S.BLOCKED:
            break
    return rig


@pytest.mark.parametrize('y', [-0.15, 0.0, 0.1])
def test_reactive_avoidance_passes_a_person_on_the_route(y):
    """E4 on_route reactive 0/20: the defaults steer before the guard stops the robot."""
    rig = drive_past(y, AvoidanceShared())
    assert rig.pose.x > 6.0
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_policy_reacting_inside_the_guard_stopping_reach_cannot_pass():
    """With safe_distance 1.0 m VFH sees the person only after the guard stopped."""
    config = FollowerConfig(**{**vars(REACTIVE), 'detour': DetourConfig(enabled=False)})
    rig = drive_past(0.0, AvoidanceShared(safe_distance=1.0, body_margin=0.10), config=config)
    assert rig.state is S.BLOCKED and rig.pose.x < 3.5


def test_defaults_leave_the_policy_room_before_the_guard():
    shared, safety, robot = AvoidanceShared(), SafetyConfig(), RobotProfile()
    lim, cruise = robot.limits, FollowerConfig().speed.cruise
    reach = robot.footprint.front + safety.stop_margin
    reach += protective_distance(cruise, lim.max_decel, lim.reaction_time, safety.human_speed)
    assert shared.safe_distance > reach
    assert shared.body_margin > safety.lateral_margin
    assert AvoidanceOut().heading_offset == 0.0
    assert np.isfinite(reach)
