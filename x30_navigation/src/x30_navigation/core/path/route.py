"""A route to follow: points, headings, stop points and arc length, with interpolation."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np

from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D, arc_lengths, wrap_angle

MIN_LENGTH = 1e-6


@dataclass(frozen=True, eq=False)
class Route:
    """A path in the map frame: yaw in radians, dwell > 0 marks a stop point."""

    xy: np.ndarray
    yaw: np.ndarray
    dwell: np.ndarray
    s: np.ndarray
    segment_yaw: np.ndarray
    curvature: np.ndarray

    @staticmethod
    def from_points(points: Sequence[RoutePoint]) -> Route:
        """Build a route; raise ValueError for fewer than two points or no length."""
        if len(points) < 2:
            raise ValueError(f'a route needs at least 2 points, got {len(points)}')
        values = np.array([(p.x, p.y, p.yaw, p.dwell_sec) for p in points], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError('a route point is not finite')
        xy = values[:, :2].copy()
        s = arc_lengths(xy)
        if s[-1] < MIN_LENGTH:
            raise ValueError('a route needs some length, every point is at the same place')
        return Route(
            xy,
            values[:, 2].copy(),
            np.maximum(values[:, 3], 0.0),
            s,
            _segment_headings(xy),
            signed_curvatures(xy),
        )

    def __len__(self) -> int:
        return len(self.xy)

    @property
    def length(self) -> float:
        """Total length (m)."""
        return float(self.s[-1])

    @property
    def stops(self) -> np.ndarray:
        """Indices of the stop points."""
        return np.flatnonzero(self.dwell > 0.0)

    def point(self, i: int) -> RoutePoint:
        """Return waypoint i."""
        return RoutePoint(
            float(self.xy[i, 0]), float(self.xy[i, 1]), float(self.yaw[i]), float(self.dwell[i])
        )

    def points(self) -> list[RoutePoint]:
        """Return every waypoint."""
        return [self.point(i) for i in range(len(self))]

    def pose(self, i: int) -> Pose2D:
        """Return waypoint i as a pose."""
        return Pose2D(float(self.xy[i, 0]), float(self.xy[i, 1]), float(self.yaw[i]))

    def segment_at(self, s: float) -> int:
        """Return the index of the segment holding arc length s (clamped)."""
        i = int(np.searchsorted(self.s, s, side='right')) - 1
        return min(max(i, 0), len(self) - 2)

    def index_at(self, s: float) -> int:
        """Return the index of the last waypoint at or before arc length s."""
        i = int(np.searchsorted(self.s, s + 1e-9, side='right')) - 1
        return min(max(i, 0), len(self) - 1)

    def point_at(self, s: float) -> np.ndarray:
        """Return the (x, y) point at arc length s, clamped to the route."""
        s = min(max(s, 0.0), self.length)
        i = self.segment_at(s)
        span = self.s[i + 1] - self.s[i]
        t = (s - self.s[i]) / span if span > MIN_LENGTH else 0.0
        return self.xy[i] + t * (self.xy[i + 1] - self.xy[i])

    def chord_heading(self, s0: float, s1: float) -> float:
        """Heading of the chord from arc length s0 to s1 (a smoothed tangent)."""
        a, b = self.point_at(s0), self.point_at(s1)
        dx, dy = b - a
        if math.hypot(dx, dy) < MIN_LENGTH:
            return float(self.segment_yaw[self.segment_at(s0)])
        return math.atan2(dy, dx)

    def shifted_point(self, s: float, offset: float) -> np.ndarray:
        """Return the point at arc length s moved offset metres to the left (+) or right."""
        p = self.point_at(s)
        if offset == 0.0:
            return p
        heading = self.chord_heading(s - 0.1, s + 0.1)
        return p + offset * np.array([-math.sin(heading), math.cos(heading)])

    def lateral(self, pose: Pose2D, s: float) -> float:
        """Signed distance (+left) of a pose from the route segment at arc length s."""
        i = self.segment_at(s)
        yaw = self.segment_yaw[i]
        dx, dy = pose.x - self.xy[i, 0], pose.y - self.xy[i, 1]
        return float(math.cos(yaw) * dy - math.sin(yaw) * dx)

    def heading_error(self, pose: Pose2D, s: float) -> float:
        """Robot heading minus the route heading at arc length s, wrapped."""
        return wrap_angle(pose.yaw - float(self.segment_yaw[self.segment_at(s)]))

    def curvature_at(self, s: float) -> float:
        """Signed curvature (1/m, +left) at arc length s."""
        return float(np.interp(s, self.s, self.curvature))

    def is_closed(self, tolerance: float) -> bool:
        """Return True when the last point is within tolerance of the first (a loop)."""
        return float(np.hypot(*(self.xy[-1] - self.xy[0]))) < tolerance


def _segment_headings(xy: np.ndarray) -> np.ndarray:
    """Heading of every segment; a zero-length segment takes its neighbour's heading."""
    d = np.diff(xy, axis=0)
    length = np.hypot(d[:, 0], d[:, 1])
    yaw = np.arctan2(d[:, 1], d[:, 0])
    good = np.flatnonzero(length >= MIN_LENGTH)
    if good.size < len(yaw):
        nearest = good[np.abs(np.arange(len(yaw))[:, None] - good[None, :]).argmin(axis=1)]
        yaw = yaw[nearest]
    return yaw


def signed_curvatures(xy: np.ndarray, span: int = 3) -> np.ndarray:
    """Signed curvature (1/m, +left) at every point from the heading change over +-span."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    n = len(xy)
    out = np.zeros(n)
    if n <= 2 * span:
        return out
    a, b, c = xy[: n - 2 * span], xy[span : n - span], xy[2 * span :]
    h1 = np.arctan2(b[:, 1] - a[:, 1], b[:, 0] - a[:, 0])
    h2 = np.arctan2(c[:, 1] - b[:, 1], c[:, 0] - b[:, 0])
    turn = (h2 - h1 + np.pi) % (2.0 * np.pi) - np.pi
    length = np.hypot(*(b - a).T) + np.hypot(*(c - b).T)
    ok = length > MIN_LENGTH
    out[span : n - span] = np.where(ok, 2.0 * turn / np.maximum(length, 1e-9), 0.0)
    return out
