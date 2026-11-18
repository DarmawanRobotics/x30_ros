"""Progress along a route: the arc length of the robot's projection, monotonic."""

from __future__ import annotations

import math

import numpy as np

from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D

WINDOW_AHEAD_M = 1.5
TRACK_HEADING_WEIGHT_M = 0.3
LOCATE_HEADING_WEIGHT_M = 1.0


class RouteProgress:
    """Monotonic progress (arc length, m) of a robot along one route."""

    def __init__(self, route: Route, s: float = 0.0):
        self.route = route
        self.s = min(max(float(s), 0.0), route.length)

    @property
    def index(self) -> int:
        """Index of the last waypoint at or before the progress."""
        return self.route.index_at(self.s)

    @property
    def remaining(self) -> float:
        """Route length left after the progress (m)."""
        return self.route.length - self.s

    def reset(self, s: float):
        """Put the progress at arc length s."""
        self.s = min(max(float(s), 0.0), self.route.length)

    def locate(self, pose: Pose2D, start: float = 0.0) -> float:
        """Find the robot on the route from arc length start on and put the progress there."""
        s, _ = self._best(pose, start, self.route.length, 0.0, LOCATE_HEADING_WEIGHT_M)
        self.s = s
        return s

    def update(self, pose: Pose2D, offset: float, max_advance: float) -> float:
        """Advance toward the robot's projection on the line offset metres beside the route."""
        end = min(self.route.length, self.s + WINDOW_AHEAD_M)
        best, _ = self._best(pose, self.s, end, offset, TRACK_HEADING_WEIGHT_M)
        self.s = min(max(best, self.s), self.s + max(0.0, max_advance))
        return self.s

    def back(self, pose: Pose2D, distance: float) -> float:
        """Put the progress back at the robot's projection, at most distance behind it."""
        lo = max(0.0, self.s - max(distance, 0.0))
        self.s, _ = self._best(pose, lo, self.s, 0.0, TRACK_HEADING_WEIGHT_M)
        return self.s

    def lateral(self, pose: Pose2D) -> float:
        """Signed distance (+left) of the robot from the route at the progress."""
        return self.route.lateral(pose, self.s)

    def _best(
        self, pose: Pose2D, lo: float, hi: float, offset: float, heading_weight: float
    ) -> tuple[float, float]:
        """Return (arc length, cost) of the best projection with s in [lo, hi]."""
        route = self.route
        idx = np.arange(route.segment_at(lo), route.segment_at(hi) + 1)
        start, length = route.s[idx], route.s[idx + 1] - route.s[idx]
        yaw = route.segment_yaw[idx]
        tangent = np.column_stack((np.cos(yaw), np.sin(yaw)))
        p = np.array([pose.x, pose.y])
        along = ((p - route.xy[idx]) * tangent).sum(axis=1)
        s = np.clip(start + np.clip(along, 0.0, length), lo, hi)
        rel = p - (route.xy[idx] + (s - start)[:, None] * tangent)
        d = rel[:, 1] * tangent[:, 0] - rel[:, 0] * tangent[:, 1]
        miss = (rel * tangent).sum(axis=1)
        turn = np.abs((pose.yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi)
        cost = np.hypot(d - offset, miss) + heading_weight * turn / math.pi
        k = int(np.argmin(cost))
        return float(s[k]), float(cost[k])
