"""Backing up after a stop point's inspection (inspection.reverse_distance_m)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import Rig, S, person
from sim import route_of, straight_route

from x30_navigation.core.avoidance import AvoidanceOut
from x30_navigation.core.config.follower import FollowerConfig, InspectionConfig
from x30_utils.io import RoutePoint

STOP = 40  # x = 4.0


def stop_route(yaw: float = 0.0):
    points = [RoutePoint(x, y) for x, y in straight_route(0.0, 8.0)]
    points[STOP] = RoutePoint(points[STOP].x, 0.0, yaw, 1.0)
    return route_of(points)


def at_stop(distance: float = 0.5, yaw: float = 0.0) -> Rig:
    """A rig dwelling at the stop point (x = 4)."""
    config = FollowerConfig(
        require_localization=False, inspection=InspectionConfig(reverse_distance_m=distance)
    )
    rig = Rig(stop_route(yaw), config)
    assert rig.follower.start(rig.pose)[0]
    rig.run(60.0, until=lambda d: d.report.state is S.DWELLING)
    assert rig.state is S.DWELLING
    return rig


def run_tracking_x(rig: Rig, seconds: float, until=None):
    """Run, recording x; rig.x0 is where the robot dwelled."""
    rig.x0, rig.xs = rig.pose.x, []
    for _ in range(int(seconds / rig.dt)):
        decision = rig.tick()
        rig.xs.append(rig.pose.x)
        if until is not None and until(decision):
            break
    return rig


def test_off_by_default_leaves_forward():
    rig = at_stop(distance=0.0)
    run_tracking_x(rig, 3.0)
    assert min(rig.xs) > rig.x0 - 0.02
    assert rig.state is S.FOLLOWING


def test_backs_up_straight_then_rejoins_and_finishes():
    rig = at_stop()
    start_yaw = rig.pose.yaw
    run_tracking_x(rig, 60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.x0 - min(rig.xs) == pytest.approx(0.5, abs=0.03)
    assert rig.state is S.GOAL_REACHED
    reversing = [d for d in rig.decisions if 'backing up from stop' in d.report.message]
    assert reversing and all(d.report.state is S.JOINING for d in reversing)
    assert all(d.command.vx <= 0.0 and d.command.vy == d.command.wz == 0.0 for d in reversing)
    assert abs(rig.pose.yaw - start_yaw) < 0.1
    assert ('DEPARTED', STOP) in rig.events()


def test_backs_up_along_a_sideways_stop_heading():
    """A stop facing a wall (yaw 90 deg): the robot backs away from it, off the route."""
    rig = at_stop(yaw=math.pi / 2)
    ys = []
    for _ in range(int(8.0 / rig.dt)):
        rig.tick()
        ys.append(rig.pose.y)
    assert min(ys) == pytest.approx(-0.5, abs=0.05)
    rig.run(60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED


def test_blocked_rear_ends_the_back_up_early():
    rig = at_stop()
    rig.obstacles = person(3.0, 0.0)  # 0.35 m behind the rear edge
    run_tracking_x(rig, 4.0, until=lambda d: 'rear blocked' in d.report.message)
    assert 'rear blocked' in rig.decisions[-1].report.message
    assert rig.x0 - min(rig.xs) < 0.1
    rig.obstacles = rig.obstacles[:0]
    rig.run(60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED


def test_blocked_way_back_reports_blocked_with_the_reason():
    """The way to the route past the stop is blocked: BLOCKED at once, the planner replans."""
    rig = at_stop()
    rig.obstacles = person(4.9, 0.0)
    run_tracking_x(rig, 10.0, until=lambda d: d.report.state is S.BLOCKED)
    report = rig.decisions[-1].report
    assert report.state is S.BLOCKED
    assert 'after backing up from stop 40 the way to waypoint 45 is blocked' in report.message
    assert rig.x0 - min(rig.xs) == pytest.approx(0.5, abs=0.03)
    rig.run(1.0)
    assert rig.state is S.BLOCKED
    rig.obstacles = rig.obstacles[:0]
    rig.run(3.0)
    assert rig.state in (S.JOINING, S.FOLLOWING)


def test_frontal_emergency_does_not_hold_the_back_up():
    """Reactive avoidance looks ahead (an inspected panel): backing up uses the 360 deg guard."""
    rig = at_stop()
    rig.avoidance = AvoidanceOut(velocity_scale=0.0, emergency=True)
    run_tracking_x(rig, 8.0)
    assert rig.x0 - min(rig.xs) == pytest.approx(0.5, abs=0.03)


def wall_stop_rig(sectors: int = 72) -> Rig:
    """E5: a route north 0.9 m from an east wall, its stop point facing the wall (yaw 0)."""
    north = math.pi / 2.0
    points = [RoutePoint(4.7, float(y), north) for y in np.arange(-2.5, 3.01, 0.25)]
    stop = int(np.argmin([abs(p.y) for p in points]))
    points[stop] = RoutePoint(4.7, 0.0, 0.0, 1.0)
    config = FollowerConfig(
        require_localization=False, inspection=InspectionConfig(reverse_distance_m=1.0)
    )
    rig = Rig(route_of(points), config)
    rig.sectors = sectors
    rig.obstacles = np.array([(5.6, y) for y in np.arange(-3.5, 4.0, 0.05)])
    assert rig.follower.start(rig.pose)[0]
    return rig


def test_rejoin_after_backing_up_from_a_wall_is_not_held():
    """E5: the rejoin turns onto the route heading first, not held short of the wall."""
    rig = wall_stop_rig()
    rig.run(120.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED
    assert not any(d.report.state is S.BLOCKED for d in rig.decisions)
    assert rig.t < 60.0
    turning = [d for d in rig.decisions if 'turning onto the route' in d.report.message]
    assert len(turning) > 10  # the first ticks still brake from backing up (slew limit)
    assert all(d.command.vx == d.command.vy == 0.0 for d in turning[5:])
    assert all(abs(d.command.wz) > 0.0 for d in turning[5:])


def test_rejoin_aims_past_something_on_the_way():
    """A blocked way to the first point past the stop takes a later clear point."""
    rig = wall_stop_rig()
    rig.run(60.0, until=lambda d: d.report.state is S.DWELLING)
    rig.obstacles = np.vstack((rig.obstacles, person(4.7, 0.5, r=0.1, n=12)))
    rig.run(120.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED
    assert not any(d.report.state is S.BLOCKED for d in rig.decisions)


def test_rejoin_waits_standing_until_a_way_clears():
    rig = wall_stop_rig()
    across = np.array([(x, 0.7) for x in np.arange(2.5, 5.6, 0.05)])
    full = np.vstack((rig.obstacles, across))
    rig.run(60.0, until=lambda d: d.report.state is S.DWELLING)
    rig.obstacles = full
    rig.run(30.0, until=lambda d: d.report.state is S.BLOCKED)
    assert 'the way to waypoint' in rig.decisions[-1].report.message
    standing = rig.pose
    rig.run(3.0)
    assert rig.state is S.BLOCKED
    assert math.hypot(rig.pose.x - standing.x, rig.pose.y - standing.y) < 0.01
    rig.obstacles = full[: -len(across)]
    rig.run(60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED
