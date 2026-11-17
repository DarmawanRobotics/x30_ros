"""Rejoining the route after backing up: a reachable route point, turning in place first."""

from __future__ import annotations

import math

import numpy as np
from rig import ROBOT, person
from sim import route_of

from x30_navigation.core.config.follower import SafetyConfig
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.follower.rejoin import JoinPlanner
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D
from x30_utils.robot import Limits, RobotProfile

NORTH = math.pi / 2.0
ROUTE = route_of([RoutePoint(4.7, float(y), NORTH) for y in np.arange(-2.5, 3.01, 0.25)])
FIRST = int(np.argmin(np.abs(ROUTE.xy[:, 1] - 1.0)))  # 1 m past the stop at y = 0
BACKED = Pose2D(3.7, 0.0, 0.0)  # backed up 1 m from the stop, still facing the wall
WALL = np.array([(5.6, y) for y in np.arange(-3.5, 4.0, 0.05)])


def plan(obstacles, robot=ROBOT, pose=BACKED):
    planner = JoinPlanner(MotionGuard(robot, SafetyConfig()), 0.04, math.radians(5.0), 3.0)
    return planner.plan(ROUTE, FIRST, pose, pose.inverse().apply(obstacles))


def test_turns_in_place_onto_the_route_heading_first():
    """E5 free: facing the wall 90 deg off the route heading, it turns first, then walks."""
    out = plan(WALL)
    assert out.reachable and out.target == FIRST and out.turn_first
    assert out.free - out.distance > SafetyConfig().stop_margin


def test_aims_at_the_first_point_with_a_clear_way():
    """Something on the way to the point 1 m past the stop: a later point is reachable."""
    out = plan(np.vstack((WALL, person(4.7, 0.5, r=0.1, n=12))))
    assert out.reachable and out.target > FIRST and out.turn_first


def test_no_clear_way_names_the_first_point():
    across = np.array([(x, 0.7) for x in np.arange(2.5, 5.6, 0.05)])
    out = plan(np.vstack((WALL, across)))
    assert not out.reachable and out.target == FIRST
    assert out.free < out.distance


def test_aligned_robot_walks_on_directly():
    out = plan(WALL, pose=Pose2D(4.2, 0.0, NORTH))
    assert out.reachable and out.target == FIRST and not out.turn_first


def test_differential_robot_drives_toward_the_point():
    diff = RobotProfile(kinematics='diff', limits=Limits(max_vy=0.0))
    out = plan(WALL, robot=diff)
    assert out.reachable and out.target == FIRST and not out.turn_first
