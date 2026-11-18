"""Where to walk back onto the route after backing up, and whether to turn in place first."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D, angle_diff
from x30_utils.safety import rotation_room

TURN_YAW = math.radians(20.0)  # further off the point's heading: turn in place first (omni)


@dataclass(frozen=True)
class JoinPlan:
    """The route point to walk to, whether to turn first, and the way to it."""

    target: int
    turn_first: bool = False
    reachable: bool = True
    free: float = math.inf
    distance: float = 0.0


@dataclass(frozen=True)
class Way:
    """One way to a point: clear or not, turning first, free distance and length (m)."""

    clear: bool
    turn_first: bool
    free: float
    distance: float


class JoinPlanner:
    """Choose the first route point the robot reaches on a clear straight way."""

    def __init__(self, guard: MotionGuard, min_speed: float, yaw_margin: float, reach: float):
        self.guard = guard
        self.footprint = guard.robot.footprint
        self.can_strafe = guard.robot.can_strafe
        self.min_speed = min_speed
        self.yaw_margin = yaw_margin
        self.reach = reach

    def plan(self, route: Route, first: int, pose: Pose2D, points: np.ndarray) -> JoinPlan:
        """Return the plan from pose with obstacle points (robot frame)."""
        blocked = None
        for i in range(first, len(route)):
            target = route.pose(i)
            if i > first and math.hypot(target.x - pose.x, target.y - pose.y) > self.reach:
                break
            way = self.way(pose, target, points)
            if way.clear:
                return JoinPlan(i, way.turn_first, True, way.free, way.distance)
            blocked = blocked or JoinPlan(first, False, False, way.free, way.distance)
        return blocked or JoinPlan(first)

    def way(self, pose: Pose2D, target: Pose2D, points: np.ndarray) -> Way:
        """Return the way from pose to target (see the module docstring)."""
        local = pose.between(target)
        distance = math.hypot(local.x, local.y)
        if self.can_strafe:
            turn = angle_diff(target.yaw, pose.yaw)
            if abs(turn) > TURN_YAW and self._turn_room(points, turn):
                return self._way(local, distance, points, (turn,), True)
            return self._way(local, distance, points, (0.0, turn), False)
        heading = math.atan2(local.y, local.x) if distance > 1e-6 else 0.0
        if abs(heading) > math.pi / 2.0:
            heading = angle_diff(heading, math.pi)  # backward, as FinalApproach drives
        way = self._way(local, distance, points, (heading,), False)
        return Way(way.clear and self._turn_room(points, heading), False, way.free, distance)

    def _way(self, local: Pose2D, distance: float, points, yaws, turn_first: bool) -> Way:
        """Return the straight way with the body turned by each of yaws (all must be clear)."""
        if distance < 1e-6:
            return Way(True, turn_first, math.inf, distance)
        clear, free = True, math.inf
        for yaw in yaws:
            c, s = math.cos(yaw), math.sin(yaw)
            turned = np.asarray(points, dtype=float).reshape(-1, 2) @ np.array([[c, -s], [s, c]])
            ux = (c * local.x + s * local.y) / distance
            uy = (-s * local.x + c * local.y) / distance
            body, band = self.guard.free(ux, uy, turned)
            # the guard still lets the robot move at the slowest speed at the point
            speed = self.guard.speed_for(body - distance, band - distance)
            clear = clear and speed >= self.min_speed
            free = min(free, band)
        return Way(clear, turn_first, free, distance)

    def _turn_room(self, points: np.ndarray, turn: float) -> bool:
        if abs(turn) < 1e-6:
            return True
        room = rotation_room(self.footprint, points, turn) - self.yaw_margin
        return room >= abs(turn)
