"""A robot held by the control mux: measured stall, the arming handshake, one lateral margin."""

from __future__ import annotations

import numpy as np
import pytest
from rig import ROBOT, Rig, S, is_zero
from sim import route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig, SafetyConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.follower.supervisor import StallWatch
from x30_utils.math import Pose2D

MUX_LATERAL_MARGIN = 0.10  # x30_control FieldConfig.lateral_margin default


def started_rig(**config):
    config.setdefault('require_localization', False)
    rig = Rig(route_of(straight_route(0.0, 12.0)), FollowerConfig(**config))
    assert rig.follower.start(rig.pose)[0]
    return rig


def test_robot_held_by_the_mux_reports_blocked():
    """E4 beside_route off 1: commanded to move but standing still: BLOCKED, not FOLLOWING."""
    rig = started_rig(safety=SafetyConfig(blocked_timeout_sec=2.0))
    for _ in range(int(3.0 / rig.dt)):
        rig.tick(move=False)
    assert rig.commands[-1].vx > 0.1
    assert rig.state is S.BLOCKED
    rig.run(2.0)
    assert rig.state is S.FOLLOWING


def test_moving_robot_is_never_blocked():
    rig = started_rig(safety=SafetyConfig(blocked_timeout_sec=1.0))
    rig.run(6.0)
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_stall_watch_counts_only_standing_time():
    watch = StallWatch(0.1, 10.0)
    assert watch.update(Pose2D(), 1.0, True) == 0.0
    assert watch.update(Pose2D(0.05, 0.0, 0.0), 1.0, True) == pytest.approx(1.0)
    assert watch.update(Pose2D(0.12, 0.0, 0.0), 1.0, True) == 0.0
    assert watch.update(Pose2D(0.12, 0.0, 0.3), 1.0, True) == 0.0
    assert watch.update(Pose2D(0.12, 0.0, 0.3), 1.0, False) == 0.0
    assert watch.update(None, 1.0, True) == 0.0


def test_disarmed_navigation_sends_neutral_until_armed():
    """E4: a follower started right after the mux (or a mux respawn) gets armed and moves."""
    rig = started_rig()
    rig.follower.on_control(estop=False, remote=False, armed=False)
    rig.run(1.0)
    assert rig.state is S.PAUSED
    assert rig.decisions[-1].report.pause_reasons == ('disarmed',)
    assert all(is_zero(d.command) for d in rig.decisions)
    rig.follower.on_control(estop=False, remote=False, armed=True)
    rig.run(2.0)
    assert rig.state is S.FOLLOWING and rig.commands[-1].vx > 0.1


def test_disarmed_in_remote_or_estop_is_not_a_reason_of_its_own():
    rig = started_rig()
    rig.follower.on_control(estop=False, remote=True, armed=False)
    rig.run(0.2)
    assert rig.decisions[-1].report.pause_reasons == ('remote',)


def test_guard_margin_covers_the_mux_corridor():
    """E4: the guard never plans a pass the mux refuses (same lateral margin by default)."""
    assert SafetyConfig().lateral_margin >= MUX_LATERAL_MARGIN
    guard = MotionGuard(ROBOT, SafetyConfig())
    hw = ROBOT.footprint.half_width
    inside_mux_corridor = np.array([[1.5, hw + 0.08]])
    assert guard.limit(Command(0.4), inside_mux_corridor, False).clearance < 2.0
    assert SafetyConfig(lateral_margin=0.05).check_control(MUX_LATERAL_MARGIN)
    assert not SafetyConfig().check_control(MUX_LATERAL_MARGIN)


def test_guard_keeps_the_separation_of_a_walking_person():
    """E4 beside_route 15: the guard adds v_h (t_r + t_stop) like the control field."""
    person = np.array([[1.2, 0.0]])
    static = MotionGuard(ROBOT, SafetyConfig(human_speed=0.0)).limit(Command(0.6), person, False)
    moving = MotionGuard(ROBOT, SafetyConfig(human_speed=0.5)).limit(Command(0.6), person, False)
    assert 0.0 < moving.command.vx < static.command.vx
    assert SafetyConfig(human_speed=0.2).check_control(MUX_LATERAL_MARGIN, 0.5)
    assert not SafetyConfig().check_control(MUX_LATERAL_MARGIN, SafetyConfig().human_speed)
