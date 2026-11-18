"""Backing up after the inspection at a stop point (inspection.reverse_distance_m > 0)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.rejoin import JoinPlan, JoinPlanner
from x30_navigation.core.follower.state import FollowerState, Inputs
from x30_utils.math import Pose2D, angle_diff

S = FollowerState
DONE_TOLERANCE_M = 0.01
TIMEOUT_FACTOR = 3.0
TIMEOUT_EXTRA_SEC = 5.0


@dataclass
class Reverse:
    """One back-up: where it started, how far it goes, how long it may take."""

    start: Pose2D | None = None
    distance: float = 0.0
    seconds: float = 0.0
    stop: int = 0
    first: int = 0  # first route point the rejoin may aim at
    turn_first: bool = False  # turning onto the route heading before walking on
    waiting: bool = False  # no clear way to the route yet: standing, planning again

    @property
    def active(self) -> bool:
        """True while backing up."""
        return self.start is not None

    def begin(self, pose: Pose2D, distance: float, stop: int):
        """Start backing up from pose after stop point stop."""
        self.end()
        self.start, self.distance, self.seconds, self.stop = pose, distance, 0.0, stop

    def travelled(self, pose: Pose2D) -> float:
        """Return how far the robot moved backward along its starting heading (m)."""
        if self.start is None:
            return 0.0
        dx, dy = pose.x - self.start.x, pose.y - self.start.y
        return -(dx * math.cos(self.start.yaw) + dy * math.sin(self.start.yaw))

    def end(self):
        """Stop backing up and forget the rejoin."""
        self.start = None
        self.turn_first = self.waiting = False


class ReverseSteps:
    """Follower steps of the back-up (a mixin of Follower, like Phases)."""

    def _begin_reverse(self, pose: Pose2D) -> bool:
        """Start backing up after a stop point when configured; True when it started."""
        distance = self.config.inspection.reverse_distance_m
        if distance <= 0.0:
            return False
        self.reverse.begin(pose, distance, self.target)
        self._set_phase(S.JOINING)
        return True

    def _reversing(self, inputs: Inputs, pose: Pose2D):
        """Back up until the distance, a blocked rear or the timeout; then rejoin."""
        cfg = self.config.inspection
        rev = self.reverse
        rev.seconds += inputs.dt
        travelled = rev.travelled(pose)
        command = Command(-cfg.reverse_speed, 0.0, 0.0)
        rear = self.guard.limit(command, inputs.scan.guard_points, inputs.scan.blocked)
        timeout = TIMEOUT_FACTOR * rev.distance / cfg.reverse_speed + TIMEOUT_EXTRA_SEC
        note = ''
        if travelled >= rev.distance - DONE_TOLERANCE_M:
            note = f'backed up {travelled:.2f} m'
        elif rear.stalled:
            note = f'rear blocked after {max(travelled, 0.0):.2f} m'
        elif rev.seconds > timeout:
            note = f'back-up timeout after {max(travelled, 0.0):.2f} m'
        if note:
            rev.end()
            return self._rejoin(inputs, pose, note)
        return command, f'backing up from stop {self.target}: {travelled:.2f} / {rev.distance} m'

    def _rejoin(self, inputs: Inputs, pose: Pose2D, note: str):
        """Aim at the first route point past the stop with a clear way; BLOCKED without one."""
        route, rev = self.tracker.route, self.reverse
        ahead = float(route.s[rev.stop]) + rev.distance
        rev.first = min(int(np.searchsorted(route.s, ahead - 1e-9)), len(route) - 1)
        self.timer = 0.0
        points = inputs.scan.guard_points
        self._take_plan(self._join_planner().plan(route, rev.first, pose, points))
        if rev.waiting:
            self.supervisor.force_blocked(pose)
        return Command(), f'{note}: joining the route at point {self.target}'

    def _rejoining(self, inputs: Inputs, pose: Pose2D):
        """Wait for a clear way, then turn onto the route heading; None: walk on (joining)."""
        rev = self.reverse
        if rev.waiting:
            route, points = self.tracker.route, inputs.scan.guard_points
            self._take_plan(self._join_planner().plan(route, rev.first, pose, points))
            if rev.waiting:
                return Command(), f'waiting for a clear way to point {self.target}'
        if rev.turn_first:
            error = angle_diff(self.tracker.route.yaw[self.target], pose.yaw)
            if abs(error) > math.radians(self.config.approach.yaw_tolerance_deg):
                self.timer += inputs.dt
                rate = self.approach.yaw_rate(error)
                return Command(0.0, 0.0, rate), f'turning onto the route at point {self.target}'
            rev.turn_first = False
        return None

    def _take_plan(self, plan: JoinPlan):
        """Aim at the planned point (the first point, standing, when no way is clear)."""
        rev = self.reverse
        self.target, rev.turn_first, rev.waiting = plan.target, plan.turn_first, not plan.reachable
        self.approach.reset()
        if rev.waiting:
            self.block_note = (
                f'after backing up from stop {rev.stop} the way to waypoint {plan.target} is'
                f' blocked ({plan.free:.2f} of {plan.distance:.2f} m free)'
            )
        else:
            self.block_note = ''

    def _join_planner(self) -> JoinPlanner:
        cfg = self.config
        reach = max(cfg.join.max_distance, 2.0 * cfg.inspection.reverse_distance_m)
        yaw_margin = math.radians(cfg.safety.yaw_margin_deg)
        return JoinPlanner(self.guard, cfg.approach.min_speed, yaw_margin, reach)
