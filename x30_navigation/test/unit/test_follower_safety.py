"""Follower safety: zero on every hold, pause reasons, TF age, blind scans, guard, shaper."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import ROBOT, Rig, S, is_zero, person
from sim import route_of, straight_route

from x30_navigation.core.avoidance import AvoidanceOut
from x30_navigation.core.config.follower import FollowerConfig, SafetyConfig, SpeedConfig
from x30_navigation.core.config.shift import PathShiftConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.command import CommandShaper
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.follower.pause import PauseReasons
from x30_navigation.core.follower.state import Inputs, PoseInput
from x30_navigation.core.follower.supervisor import Problem, Supervisor
from x30_utils.math import Pose2D
from x30_utils.robot import Footprint, Limits, RobotProfile


def moving_rig(**config):
    config = {k: v for k, v in config.items() if v is not None}
    rig = Rig(
        route_of(straight_route(0.0, 10.0)), FollowerConfig(require_localization=False, **config)
    )
    rig.follower.start(rig.pose)
    rig.run(3.0)
    assert rig.commands[-1].vx > 0.1
    return rig


@pytest.mark.parametrize(
    'hold',
    ['estop', 'remote', 'operator', 'localization', 'battery', 'stale_scan', 'blind', 'stale_tf'],
)
def test_every_hold_publishes_zero_every_tick(hold):
    """F18: stop / e-stop / stale / pause give an immediate zero, every tick."""
    rig = moving_rig()
    follower = rig.follower
    if hold == 'estop':
        follower.on_control(estop=True, remote=False)
    elif hold == 'remote':
        follower.on_control(estop=False, remote=True)
    elif hold == 'operator':
        follower.pause()
    elif hold == 'localization':
        follower.config = FollowerConfig(require_localization=True)
        follower.on_localization(False)
    elif hold == 'battery':
        follower.on_battery(0.05)
    elif hold == 'stale_scan':
        rig.scan_age = 2.0
    elif hold == 'blind':
        rig.blind = True
    elif hold == 'stale_tf':
        rig.pose_age = 2.0
    for _ in range(5):
        assert is_zero(rig.tick().command)


def test_estop_is_sticky_and_remote_is_transient():
    """F16 / DESIGN: estop clears only by resume after the release; remote by itself."""
    rig = moving_rig()
    rig.follower.on_control(estop=True, remote=False)
    rig.run(0.5)
    assert rig.state is S.PAUSED and rig.decisions[-1].report.pause_reasons == ('estop',)
    rig.follower.on_control(estop=False, remote=False)
    rig.run(0.5)
    assert rig.state is S.PAUSED
    rig.follower.on_control(estop=True, remote=False)
    assert not rig.follower.resume()[0]
    rig.follower.on_control(estop=False, remote=False)
    assert rig.follower.resume()[0]
    rig.follower.on_control(estop=False, remote=True)
    rig.run(0.5)
    assert rig.decisions[-1].report.pause_reasons == ('remote',)
    rig.follower.on_control(estop=False, remote=False)
    rig.run(2.0)
    assert rig.state is S.FOLLOWING and rig.commands[-1].vx > 0.05


def test_relocalization_unpauses():
    """F16: "paused until relocalized" clears again once localization is back."""
    rig = moving_rig()
    rig.follower.config = FollowerConfig(require_localization=True)
    rig.follower.on_localization(False)
    rig.run(0.5)
    assert rig.decisions[-1].report.pause_reasons == ('localization',)
    rig.follower.on_localization(True)
    rig.run(2.0)
    assert rig.state is S.FOLLOWING and rig.commands[-1].vx > 0.05


def test_tf_status_messages():
    """TF_STARTING before the first pose, TF_STALE after, TF_CLOCK for a pose from the future."""
    sup = Supervisor(FollowerConfig())
    inputs = Inputs(0.0, 0.05)
    assert sup.pose(inputs).problem is Problem.TF_STARTING
    assert sup.pose(Inputs(0.0, 0.05, PoseInput(Pose2D(), 0.0))).problem is None
    assert sup.pose(Inputs(0.0, 0.05, PoseInput(Pose2D(), 1.0))).problem is Problem.TF_STALE
    assert sup.pose(inputs).problem is Problem.TF_STALE
    assert sup.pose(Inputs(0.0, 0.05, PoseInput(Pose2D(), -0.5))).problem is Problem.TF_CLOCK


def test_slightly_old_pose_is_predicted_with_the_twist():
    """TF age in [compensate, timeout]: predict forward with the twist instead of stopping."""
    sup = Supervisor(
        FollowerConfig(safety=SafetyConfig(tf_compensate_sec=0.1, tf_timeout_sec=0.5))
    )
    fresh = sup.pose(Inputs(0.0, 0.05, PoseInput(Pose2D(1.0, 0.0, 0.0), 0.05), twist=Command(0.4)))
    assert not fresh.predicted and fresh.pose.x == 1.0
    old = sup.pose(Inputs(0.0, 0.05, PoseInput(Pose2D(1.0, 0.0, 0.0), 0.3), twist=Command(0.4)))
    assert old.predicted and old.pose.x == pytest.approx(1.12)


def test_blind_scan_blocks_after_the_timeout():
    """F1/F18: a blind scan never reads as clear; it holds and becomes BLOCKED."""
    rig = moving_rig(safety=SafetyConfig(blocked_timeout_sec=1.0))
    rig.blind = True
    rig.run(1.5)
    assert rig.state is S.BLOCKED and all(is_zero(c) for c in rig.commands[-25:])
    rig.blind = False
    rig.run(1.0)
    assert rig.state is S.FOLLOWING


@pytest.mark.parametrize('mode', ['path_shift', 'slide_off', 'reactive'])
def test_emergency_holds_for_the_release_time(mode):
    """The forward motion stops at once; path shift (emergency_slide) only holds it forward."""
    shift = PathShiftConfig(emergency_slide=mode != 'slide_off')
    rig = moving_rig(path_shift=shift, avoidance_mode='reactive' if mode == 'reactive' else None)
    rig.avoidance = AvoidanceOut(emergency=True)
    first = rig.tick()
    assert first.command.vx == 0.0 and first.report.message.startswith('EMERGENCY')
    assert is_zero(first.command) or mode == 'path_shift'
    rig.avoidance = AvoidanceOut()
    rig.run(0.5)
    assert all(c.vx == 0.0 for c in rig.commands[-10:])
    assert all(is_zero(c) for c in rig.commands[-10:]) or mode == 'path_shift'
    rig.run(1.5)
    assert rig.commands[-1].vx > 0.0


def pole_rig(**config):
    """A 10 cm pole on the route 0.65 m ahead of the robot: inside the emergency cone."""
    config = FollowerConfig(require_localization=False, **config)
    rig = Rig(route_of(straight_route(0.0, 12.0)), config, pose=Pose2D(5.3, 0.0, 0.0))
    rig.obstacles = person(6.0, 0.0, r=0.05, n=8)
    rig.emergency_zone = (0.75, 60.0)  # obstacle_avoidance_node's shared.emergency_*
    assert rig.frontal() and rig.follower.start(rig.pose)[0]
    return rig


def test_path_shift_slides_out_of_an_emergency_and_passes():
    """Field (L1W): a pole in the emergency cone: the robot slides out of it and passes."""
    rig = pole_rig()
    held = []
    for _ in range(int(25.0 / rig.dt)):
        frontal = rig.frontal()
        decision = rig.tick()
        if frontal:
            held.append(decision.command)
    assert held and all(c.vx == 0.0 for c in held)  # never toward the cone, never backward
    assert any(abs(c.vy) > 0.05 for c in held)
    assert rig.pose.x > 7.0
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_emergency_slide_off_holds_everything():
    rig = pole_rig(path_shift=PathShiftConfig(emergency_slide=False))
    rig.run(12.0)
    assert all(is_zero(c) for c in rig.commands)
    assert rig.state is S.BLOCKED and rig.pose.x == 5.3


def test_a_held_slide_goes_on_parallel():
    """The guard holds the way to the offset: the robot goes on parallel at its offset."""
    rig = Rig(route_of(straight_route(0.0, 12.0)), pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = person(4.0, 0.30 + 0.25)  # 2 cm beside the left side
    assert rig.follower.start(rig.pose)[0]
    follower, inputs = rig.follower, rig.inputs()
    follower.tracker.shift.offset = 0.3  # committed to the left: the slide goes toward them
    toward = Command(0.3, 0.15, 0.0)
    assert follower.guard.limit(toward, inputs.scan.guard_points, False).stalled
    guarded = follower._guard(toward, inputs, rig.pose)
    assert not guarded.stalled
    assert guarded.command.vx == pytest.approx(0.3, abs=1e-3) and abs(guarded.command.vy) < 1e-3


def test_guard_checks_every_direction_of_the_footprint():
    """F7: an obstacle behind or beside stops a backward or sideways command (360 deg)."""
    guard = MotionGuard(ROBOT, SafetyConfig(stop_margin=0.2))
    behind = np.array([[-0.55, 0.0]])
    assert guard.limit(Command(-0.3), behind, False).command.vx > -0.05
    assert guard.limit(Command(0.3), behind, False).command.vx == pytest.approx(0.3)
    beside = np.array([[0.0, 0.42]])
    assert guard.limit(Command(0.0, 0.3), beside, False).command.vy < 0.05
    corner = np.array([[0.3, 0.33]])
    assert abs(guard.limit(Command(0.0, 0.0, 1.0), corner, False).command.wz) < 1.0
    assert guard.limit(Command(0.3), np.zeros((0, 2)), True).stalled


def test_joining_is_guarded_by_the_scan():
    """F7: walking onto the route sideways stops for an obstacle in that direction."""
    rig = Rig(
        route_of(straight_route(0.0, 6.0)),
        FollowerConfig(require_localization=False),
        pose=Pose2D(3.0, 1.5, 0.0),
    )
    assert rig.follower.start(rig.pose)[0]
    rig.obstacles = person(3.0, 0.7)
    rig.run(5.0)
    assert rig.state is S.JOINING and rig.pose.y > 1.0


def test_shaper_envelope_slew_and_nan():
    robot = RobotProfile(
        kinematics='diff',
        footprint=Footprint(),
        limits=Limits(max_vx=0.5, max_vy=0.0, max_wz=1.0, max_accel=1.0, max_decel=2.0),
    )
    shaper = CommandShaper(robot, 1.0)
    assert shaper.envelope(Command(1.0, 0.3, 5.0)) == Command(0.5, 0.0, 1.0)
    assert shaper.shape(Command(math.nan, 0.0, math.inf), 0.1) == Command(0.0, 0.0, 0.0)
    assert shaper.shape(Command(0.5), 0.1).vx == pytest.approx(0.1)
    shaper.shape(Command(0.5), 1.0)
    assert shaper.shape(Command(0.0), 0.1).vx == pytest.approx(0.3)
    assert shaper.stop() == Command()


def test_config_ranges_and_cross_checks():
    """F11: inconsistent numbers are rejected by the config validators."""
    assert SpeedConfig(cruise=0.2, min_speed=0.3).validate()
    assert SafetyConfig(tf_compensate_sec=1.0, tf_timeout_sec=0.5).validate()
    assert FollowerConfig(controller='nope').validate()
    assert not FollowerConfig().validate()


def test_pause_reason_names():
    pauses = PauseReasons()
    pauses.pause()
    pauses.on_battery(True)
    assert pauses.names() == ('battery', 'operator')
    assert pauses.resume() == (True, 'still paused: battery')
