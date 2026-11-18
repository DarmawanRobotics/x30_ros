"""Plan a walkable path on a costmap from the robot's pose to a goal pose."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from x30_navigation.core.config.planner import SearchConfig
from x30_navigation.core.path.processing import headings, resample, smooth
from x30_navigation.core.planning.astar import SearchTimeoutError, astar, nearest_free
from x30_navigation.core.planning.costmap import Costmap
from x30_navigation.core.planning.footprint import FootprintChecker
from x30_navigation.core.planning.line import shortcut
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D, arc_lengths


@dataclass(frozen=True)
class Plan:
    """Outcome of planning: waypoints in the map frame (yaw in radians) and the goal reached."""

    ok: bool
    message: str
    waypoints: list[RoutePoint] = field(default_factory=list)
    length: float = 0.0
    goal: Pose2D | None = None


class Planner:
    """Plan on one costmap with the robot's footprint."""

    def __init__(self, costmap: Costmap, checker: FootprintChecker, config: SearchConfig):
        self.costmap = costmap
        self.checker = checker
        self.config = config

    def plan(self, start: Pose2D, goal: Pose2D) -> Plan:
        """Plan from start to goal; the plan's goal is where the path really ends."""
        cm = self.costmap
        start_cell, goal_cell = cm.cell(start.x, start.y), cm.cell(goal.x, goal.y)
        if not cm.inside(*start_cell):
            return Plan(False, f'the robot ({start.x:.2f}, {start.y:.2f}) is outside the map')
        if not cm.inside(*goal_cell):
            return Plan(False, f'the goal ({goal.x:.2f}, {goal.y:.2f}) is outside the map')
        free_start = nearest_free(cm.cost, start_cell, self._cells(self.config.start_escape))
        if free_start is None:
            return Plan(
                False,
                f'the robot is boxed in: no free cell within {self.config.start_escape:.1f} m'
                ' (is it localized on this map?)',
            )
        snapped = self._snap(goal, goal_cell)
        if snapped is None:
            return Plan(False, f'the goal ({goal.x:.2f}, {goal.y:.2f}) is inside an obstacle')
        return self._search(start, free_start, snapped, goal)

    def _cells(self, metres: float) -> int:
        return math.ceil(metres / self.costmap.resolution)

    def _snap(self, goal: Pose2D, cell: tuple[int, int]) -> Pose2D | None:
        """Return the goal, or the nearest pose within the tolerance where the robot fits."""
        if not self.costmap.lethal(*cell) and not self.checker.collides(goal):
            return goal

        def fits(c):
            x, y = self.costmap.center(*c)
            return not self.checker.collides(Pose2D(x, y, goal.yaw))

        free = nearest_free(self.costmap.cost, cell, self._cells(self.config.goal_tolerance), fits)
        if free is None:
            return None
        x, y = self.costmap.center(*free)
        return Pose2D(x, y, goal.yaw)

    def _search(self, start: Pose2D, free_start, goal: Pose2D, asked: Pose2D) -> Plan:
        """Search, check the footprint along the result and search again where it collides."""
        planner = self
        for _ in range(self.config.footprint_retries + 1):
            try:
                cells = astar(
                    planner.costmap.cost,
                    free_start,
                    planner.costmap.cell(goal.x, goal.y),
                    self.config.heuristic_weight,
                    timeout_sec=self.config.timeout_sec,
                )
            except SearchTimeoutError as error:
                return Plan(False, f'no path found in time ({error})')
            if cells is None:
                return Plan(False, 'no path: the goal cannot be reached on this map')
            route = planner._route(start, free_start, cells, goal)
            poses = [Pose2D(p.x, p.y, p.yaw) for p in route]
            skip = self._escape_points(start, free_start)
            hit = self.checker.first_collision(poses, skip)
            if hit is None:
                return self._result(route, goal, asked)
            planner = Planner(
                planner.costmap.with_lethal([planner.costmap.cell(poses[hit].x, poses[hit].y)]),
                self.checker,
                self.config,
            )
        return Plan(False, 'no path where the robot footprint fits (narrow passage?)')

    def _escape_points(self, start: Pose2D, free_start) -> int:
        """Return how many first waypoints may overlap obstacles (walking out of them)."""
        if free_start == self.costmap.cell(start.x, start.y):
            return 1
        return max(2, int(self.config.start_escape / self.config.spacing) + 1)

    def _route(self, start: Pose2D, free_start, cells, goal: Pose2D) -> list[RoutePoint]:
        """Turn search cells into spaced, smoothed waypoints with headings."""
        cm = self.costmap
        cells = shortcut(cm.cost, cells)
        points = [(start.x, start.y)]
        if free_start != cm.cell(start.x, start.y):
            points.append(cm.center(*free_start))
        points += [cm.center(*c) for c in cells[1:-1]]
        points.append((goal.x, goal.y))
        xy = np.array(points, dtype=float)
        if arc_lengths(xy)[-1] < 1e-6:
            xy = np.array([(start.x, start.y), (goal.x, goal.y)])
        dense = resample(xy, self.config.spacing)
        smoothed = smooth(dense, self.config.smooth_weight_data, self.config.smooth_weight_smooth)
        skip = self._escape_points(start, free_start)
        if len(dense) > 2 and all(math.isfinite(cm.cost_at(x, y)) for x, y in smoothed[skip:]):
            dense = smoothed
        yaws = headings(dense) if len(dense) > 1 else np.array([goal.yaw])
        if len(yaws) != len(dense):
            raise ValueError(f'{len(yaws)} headings for {len(dense)} points')
        route = [RoutePoint(float(x), float(y), float(t)) for (x, y), t in zip(dense, yaws)]
        last = route[-1]
        route[-1] = RoutePoint(last.x, last.y, goal.yaw)
        return route

    def _result(self, route: list[RoutePoint], goal: Pose2D, asked: Pose2D) -> Plan:
        length = float(arc_lengths(np.array([(p.x, p.y) for p in route]))[-1])
        note = '' if goal == asked else ' (goal moved out of an obstacle)'
        return Plan(True, f'{length:.1f} m{note}', route, length, goal)


def remaining_length(path_xy: np.ndarray, x: float, y: float) -> float:
    """Return the path length after the projection of (x, y), plus the distance to it."""
    path = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    if len(path) == 0:
        return 0.0
    if len(path) == 1:
        return float(math.hypot(path[0, 0] - x, path[0, 1] - y))
    s = arc_lengths(path)
    a, ab = path[:-1], np.diff(path, axis=0)
    length2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
    t = np.clip(((np.array([x, y]) - a) * ab).sum(axis=1) / length2, 0.0, 1.0)
    foot = a + t[:, None] * ab
    gap = np.hypot(foot[:, 0] - x, foot[:, 1] - y)
    k = int(np.argmin(gap))
    return float(gap[k] + s[-1] - (s[k] + t[k] * np.sqrt(length2[k])))
