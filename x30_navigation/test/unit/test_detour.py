"""Detours: turn in place toward a gap and drive through it when sliding does not get on."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import ROBOT, Rig, S, person
from sim import block, route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.config.shift import DetourConfig, PathShiftConfig
from x30_navigation.core.follower.detour import Detour, DetourPlanner
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.shift.path_shift import PathShift
from x30_utils.math import Pose2D
from x30_utils.robot import Limits, RobotProfile

CONFIG = DetourConfig()
ROUTE = route_of(straight_route(0.0, 14.0))
NO_STRAFE = RobotProfile(
    footprint=ROBOT.footprint,
    limits=Limits(max_vx=0.8, max_vy=0.0, max_wz=1.0, max_accel=0.5, max_decel=0.6),
)


def pocket(box_y: float = 0.30):
    """Walls 0.25 m beside both sides, a box ahead on the route, open ahead-left."""
    return np.vstack(
        [block(2.0, 4.1, 0.53, 1.2), block(2.0, 5.6, -1.2, -0.53), block(5.0, 6.0, -0.53, box_y)]
    )


START = Pose2D(4.2, 0.0, 0.0)


def plan(points, pose=START, limit=1.5, config=CONFIG):
    """Plan from pose on the straight route, obstacles in the map frame."""
    guard = MotionGuard(ROBOT, FollowerConfig().safety)
    corridor = PathShift(ROUTE, ROBOT, PathShiftConfig()).corridor
    local = pose.inverse().apply(np.asarray(points, float))
    return DetourPlanner(guard, math.radians(5.0)).plan(
        config, corridor, 0.0, pose.x, pose.y, pose, local, limit
    )


def test_a_person_ahead_is_passed_on_a_clear_straight_run():
    run = plan(person(5.4, 0.0))
    assert run is not None and math.radians(20.0) <= abs(run.angle) <= math.radians(75.0)
    assert run.progress >= CONFIG.min_progress
    guard = MotionGuard(ROBOT, FollowerConfig().safety)
    c, s = math.cos(run.turn), math.sin(run.turn)
    local = Pose2D(4.2, 0.0, 0.0).inverse().apply(person(5.4, 0.0))
    body, band = guard.free(1.0, 0.0, local @ np.array([[c, -s], [s, c]]))
    assert body > run.length + guard.config.stop_margin
    assert band > run.length + guard.config.pass_margin


def test_a_dead_end_has_no_detour():
    wall = np.vstack([block(5.0, 5.4, -3.0, 3.0), block(2.0, 5.4, 1.6, 2.0)])
    assert plan(wall) is None


def test_a_run_never_ends_further_from_the_route_than_the_limit():
    run = plan(person(5.4, 0.0), limit=0.6)
    assert run is not None
    assert abs(math.sin(run.angle)) * run.length <= 0.6 + 1e-6


def test_no_room_to_turn_no_detour():
    """Walls 0.12 m beside the whole body: no room to turn, so no detour."""
    tight = np.vstack(
        [block(2.0, 4.1, 0.40, 1.2), block(2.0, 5.6, -1.2, -0.40), block(5.0, 6.0, -0.4, 0.25)]
    )
    assert plan(tight) is None


def test_a_run_must_take_the_robot_along_the_route():
    """Only sideways (and back) is free: no run makes min_progress."""
    ahead = np.vstack([block(4.9, 5.3, -3.0, 3.0)])
    assert plan(ahead, config=DetourConfig(min_progress=0.3)) is None


def test_in_a_pocket_the_robot_turns_toward_the_gap_and_drives_through():
    """Field (L1W): too close to slide, room to turn: it turns, drives the gap, returns."""
    rig = Rig(ROUTE, pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = pocket()
    assert rig.follower.start(rig.pose)[0]
    rig.run(30.0)
    messages = [d.report.message for d in rig.decisions]
    assert any(m.startswith('detour: turning') for m in messages)
    assert any(m.startswith('detour:') and ' m on at ' in m for m in messages)
    assert rig.pose.x > 9.0 and abs(rig.pose.y) < 0.1
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_without_detours_the_pocket_holds_it():
    config = FollowerConfig(require_localization=False, detour=DetourConfig(enabled=False))
    rig = Rig(ROUTE, config, pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = pocket()
    assert rig.follower.start(rig.pose)[0]
    rig.run(15.0)
    assert rig.state is S.BLOCKED and rig.pose.x < 4.5


def test_a_robot_that_cannot_strafe_drives_round_a_person():
    """No sideways motion: path shift steered and crept along the person; it detours."""
    rig = Rig(ROUTE, robot=NO_STRAFE)
    rig.obstacles = person(4.0, 0.0)
    assert rig.follower.start(rig.pose)[0]
    rig.run(35.0)
    assert any(d.report.message.startswith('detour:') for d in rig.decisions)
    assert rig.pose.x > 8.0 and abs(rig.pose.y) < 0.1


def test_the_detour_ends_when_the_way_opens():
    """Crowds: someone steps aside while it turns: it follows the route again at once."""
    rig = Rig(ROUTE, pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = pocket()
    assert rig.follower.start(rig.pose)[0]
    rig.run(10.0, until=lambda d: d.report.message.startswith('detour: turning'))
    rig.obstacles = pocket()[: -len(block(5.0, 6.0, -0.53, 0.30))]  # the box went away
    rig.tick()
    assert not rig.follower.detour.active
    rig.run(15.0)
    assert rig.pose.x > 8.0 and abs(rig.pose.y) < 0.1


def test_near_a_stop_point_it_waits_instead():
    route = route_of(straight_route(0.0, 8.0), stops={52: 1.0})
    rig = Rig(route, pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = pocket()
    assert rig.follower.start(rig.pose)[0]
    rig.run(10.0)
    assert not any(d.report.message.startswith('detour') for d in rig.decisions)


@pytest.mark.parametrize('seconds', [0.5, 1.0])
def test_creeping_counts_as_stuck_a_slide_does_not(seconds):
    creeping, sliding = Detour(), Detour()
    n = round(seconds / 0.05)
    for k in range(n + 1):
        t = 0.05 * k
        stuck = creeping.stuck(t, Pose2D(0.05 * t, 0.0, 0.0), seconds)  # 5 cm/s on: creeping
        assert stuck is (k == n)
        assert not sliding.stuck(t, Pose2D(0.0, 0.25 * t, 0.0), seconds)  # 25 cm/s sideways
