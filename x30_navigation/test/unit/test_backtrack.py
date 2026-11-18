"""Backing up along the driven path when the robot gets no further along the route."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import ROBOT, Rig, S, person
from sim import block, route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.config.shift import BacktrackConfig
from x30_navigation.core.follower.backtrack import StuckWatch, Trail, WayBack
from x30_utils.math import Pose2D
from x30_utils.robot import Limits, RobotProfile

ROUTE = route_of(straight_route(0.0, 14.0))
FAST = BacktrackConfig(after_sec=6.0)
NO_STRAFE = RobotProfile(
    footprint=ROBOT.footprint,
    limits=Limits(max_vx=0.8, max_vy=0.0, max_wz=1.0, max_accel=0.5, max_decel=0.6),
)
BACKING = 'backing up along the driven path'


def dead_end():
    """Walls 0.12 m beside the body from x 2.5 on (no slide, no turn), a box across at x 6."""
    return np.vstack(
        [block(2.5, 6.4, 0.40, 0.6), block(2.5, 6.4, -0.6, -0.40), block(6.0, 6.4, -0.40, 0.40)]
    )


def config(**backtrack):
    """The follower configuration of the tests, backtrack changed."""
    values = {'after_sec': FAST.after_sec, **backtrack}
    return FollowerConfig(require_localization=False, backtrack=BacktrackConfig(**values))


def touches(rig) -> bool:
    """Return True when an obstacle point is inside the footprint."""
    if not len(rig.obstacles):
        return False
    local = rig.pose.inverse().apply(rig.obstacles)
    fp = rig.robot.footprint
    x, y = local[:, 0], local[:, 1]
    return bool(np.any((x <= fp.front) & (x >= -fp.rear) & (np.abs(y) <= fp.half_width)))


def drive(rig, seconds: float, until=None):
    """Run the rig, asserting it never touches an obstacle; return the decisions."""
    out = []
    trace = rig.__dict__.setdefault('trace', [])
    for _ in range(int(seconds / rig.dt)):
        out.append(rig.tick())
        trace.append((rig.pose.x, rig.pose.y))
        assert not touches(rig), f'touched an obstacle at {rig.pose}'
        if until is not None and until(out[-1]):
            break
    return out


def backing(decision) -> bool:
    return BACKING in decision.report.message


def test_a_dead_end_backs_up_three_times_then_waits():
    rig = Rig(ROUTE, config())
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    decisions = drive(rig, 90.0)
    starts = [
        i
        for i in range(1, len(decisions))
        if backing(decisions[i]) and not backing(decisions[i - 1])
    ]
    assert len(starts) == 3
    assert rig.follower.backtrack.attempts == 3
    assert rig.state is S.BLOCKED
    assert 'backed up 3 times' in decisions[-1].report.message


def test_each_back_up_goes_the_distance_along_the_way_it_came():
    rig = Rig(ROUTE, config())
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 30.0, until=backing)
    start = rig.pose
    drive(rig, 20.0, until=lambda d: not backing(d))
    assert start.x - rig.pose.x == pytest.approx(1.0, abs=0.08)
    assert abs(rig.pose.y) < 0.05 and abs(rig.pose.yaw) < math.radians(3.0)
    assert rig.follower.phase is S.FOLLOWING


def test_once_the_way_clears_it_goes_on_to_the_goal():
    rig = Rig(ROUTE, config())
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 30.0, until=backing)
    rig.obstacles = dead_end()[: -len(block(6.0, 6.4, -0.40, 0.40))]  # the box went away
    drive(rig, 60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED


def test_a_blocked_rear_ends_the_back_up_without_touching():
    rig = Rig(ROUTE, config())
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 30.0, until=backing)
    behind = rig.pose.x - ROBOT.footprint.rear - 0.45
    rig.obstacles = np.vstack([dead_end(), block(behind - 0.2, behind, -0.40, 0.40)])
    decisions = drive(rig, 15.0, until=lambda d: not backing(d))
    assert 'rear blocked' in decisions[-1].report.message
    assert rig.follower.phase is S.FOLLOWING


def test_it_backs_round_the_corner_it_came_round():
    corner = [*straight_route(0.0, 4.0), *[(4.0, 0.1 * i) for i in range(1, 41)]]
    route = route_of(corner)
    walls = np.vstack(
        [block(4.40, 4.6, 0.9, 2.8), block(3.4, 3.60, 0.9, 2.8), block(3.6, 4.4, 2.4, 2.8)]
    )
    rig = Rig(route, config(distance=2.5))
    rig.obstacles = walls
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 40.0, until=backing)
    path = np.array(rig.trace)
    gaps = []
    for _ in drive(rig, 40.0, until=lambda d: not backing(d)):
        x, y = rig.trace[-1]
        gaps.append(float(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y))))
    assert max(gaps) < 0.15
    assert rig.pose.y < 0.3 and rig.pose.x < 3.6


def test_a_robot_that_cannot_strafe_drives_straight_back():
    rig = Rig(ROUTE, config(), robot=NO_STRAFE)
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 30.0, until=backing)
    decisions = drive(rig, 20.0, until=lambda d: not backing(d))
    moving = [d.command for d in decisions[1:-1] if d.command is not None]
    assert moving and all(c.vy == 0.0 and c.vx <= 0.0 for c in moving)
    assert any(c.vx < 0.0 for c in moving)


def test_a_pause_does_not_count_as_stuck():
    rig = Rig(ROUTE, config())
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 15.0, until=lambda d: d.report.state is S.BLOCKED)
    rig.follower.pause()
    decisions = drive(rig, 3.0 * FAST.after_sec)
    assert not any(backing(d) for d in decisions)
    rig.follower.resume()
    decisions = drive(rig, FAST.after_sec - 1.0)
    assert not any(backing(d) for d in decisions)
    assert backing(drive(rig, 3.0, until=backing)[-1])


def test_switched_off_it_only_waits():
    rig = Rig(ROUTE, config(enabled=False))
    rig.obstacles = dead_end()
    assert rig.follower.start(rig.pose)[0]
    decisions = drive(rig, 60.0)
    assert not any(backing(d) for d in decisions)
    assert rig.state is S.BLOCKED


def test_a_person_ahead_who_leaves_needs_no_back_up():
    rig = Rig(ROUTE, config())
    rig.obstacles = person(5.0, 0.0)
    assert rig.follower.start(rig.pose)[0]
    drive(rig, 40.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED
    assert not any(backing(d) for d in rig.decisions)


def test_the_trail_keeps_spaced_poses_and_starts_over_after_a_jump():
    trail = Trail()
    for i in range(30):
        trail.add(Pose2D(0.01 * i, 0.0, 0.0))
    xs = [p.x for p in trail.poses]
    assert len(xs) >= 5 and all(b - a >= 0.05 - 1e-9 for a, b in zip(xs, xs[1:]))
    assert trail.length == pytest.approx(xs[-1])
    trail.add(Pose2D(5.0, 0.0, 0.0))
    assert len(trail.poses) == 1 and trail.length == 0.0


def test_the_way_back_follows_the_trail_and_rewinding_drops_what_was_driven():
    trail = Trail()
    for i in range(41):
        trail.add(Pose2D(0.1 * i, 0.0, 0.0))
    way = trail.way_back(Pose2D(4.0, 0.0, 0.0), 1.05)
    assert way.length == pytest.approx(1.05)
    assert way.at(way.length)[:2] == pytest.approx((2.95, 0.0))
    trail.rewind(1.05)
    assert trail.poses[-1].x == pytest.approx(2.95) and trail.length == pytest.approx(2.95)


def test_the_way_back_projects_monotonically():
    way = WayBack.straight(Pose2D(2.0, 0.0, 0.0), 1.0)
    assert way.project(Pose2D(1.6, 0.05, 0.0), 0.0) == pytest.approx((0.4, 0.05))
    assert way.project(Pose2D(1.9, 0.0, 0.0), 0.4)[0] == pytest.approx(0.4)


def test_progress_starts_the_stuck_time_over():
    watch = StuckWatch()
    assert watch.update(1.0, 0.5, 0.3) == 0.0
    assert watch.update(1.2, 0.5, 0.3) == 0.5
    assert watch.update(1.4, 0.5, 0.3) == 0.0
    watch.reset()
    assert watch.anchor is None
