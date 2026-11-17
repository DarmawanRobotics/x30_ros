"""The free corridor along a route, in route (Frenet) coordinates."""

from __future__ import annotations

import math

import numpy as np

from x30_navigation.core.path.route import Route
from x30_navigation.core.sweep import ALLOWANCE_M
from x30_utils.robot import Footprint

MARGIN_M = 0.5


class Corridor:
    """Free distances along offsets of one route for one footprint."""

    def __init__(
        self, route: Route, footprint: Footprint, look_ahead: float, clearance: float, reach: float
    ):
        self.route = route
        self.footprint = footprint
        self.look_ahead = look_ahead
        self.clearance = clearance
        self.reach = reach

    def coordinates(self, s_robot: float, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (s, d) of the points near the route from the rear edge to the look-ahead."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        route, fp = self.route, self.footprint
        if points.size == 0:
            return np.empty(0), np.empty(0)
        lo = route.segment_at(s_robot - fp.rear - MARGIN_M)
        hi = route.segment_at(s_robot + fp.front + self.look_ahead + MARGIN_M)
        idx = np.arange(lo, hi + 1)
        a, length = route.xy[idx], route.s[idx + 1] - route.s[idx]
        yaw = route.segment_yaw[idx]
        tangent = np.column_stack((np.cos(yaw), np.sin(yaw)))
        rel = points[:, None, :] - a[None, :, :]
        t_raw = (rel * tangent[None]).sum(axis=2)
        t = np.clip(t_raw, 0.0, length[None, :])
        foot = a[None] + t[..., None] * tangent[None]
        dist = np.hypot(points[:, None, 0] - foot[..., 0], points[:, None, 1] - foot[..., 1])
        k = dist.argmin(axis=1)
        rows = np.arange(len(points))
        d = tangent[k, 0] * rel[rows, k, 1] - tangent[k, 1] * rel[rows, k, 0]
        along = t[rows, k]
        raw = t_raw[rows, k]
        before = (k == 0) & (raw < 0.0)
        after = (k == len(idx) - 1) & (raw > length[k])
        corner = ~before & ~after & ((raw < 0.0) | (raw > length[k]))
        if corner.any():
            vertex = idx[k[corner]] + (raw[corner] > 0.0)
            d[corner] = self._corner_offsets(points[corner], vertex)
        along = np.where(before | after, raw, along)
        s = route.s[idx][k] + along
        near = np.abs(d) < self.reach
        if hi == len(route) - 2:
            near &= ~after
        return s[near], d[near]

    def _corner_offsets(self, points: np.ndarray, vertex: np.ndarray) -> np.ndarray:
        """Return the signed distance of points whose nearest route point is a corner (vertex)."""
        route = self.route
        rel = points - route.xy[vertex]
        side = np.zeros(len(points))
        for segment in (vertex - 1, vertex):
            yaw = route.segment_yaw[segment]
            cross = np.cos(yaw) * rel[:, 1] - np.sin(yaw) * rel[:, 0]
            side = np.where(np.abs(cross) > np.abs(side), cross, side)
        return np.copysign(np.hypot(rel[:, 0], rel[:, 1]), side)

    def free_distances(
        self, s_pts, d_pts, s_robot: float, offsets, extra=0.0, passing: bool = False
    ) -> np.ndarray:
        """Return the free distance ahead of the front edge per offset; 0 when blocked beside."""
        offsets = np.atleast_1d(np.asarray(offsets, dtype=float))
        free = np.full(offsets.shape, math.inf)
        if np.size(s_pts) == 0:
            return free
        fp = self.footprint
        width = fp.half_width + self.clearance + np.broadcast_to(extra, offsets.shape)
        apart = np.abs(d_pts[None, :] - offsets[:, None])
        band = apart < width[:, None]
        ahead = s_pts - (s_robot + fp.front)
        beside = (s_pts >= s_robot - fp.rear) & (ahead <= 0.0)
        in_range = (ahead > 0.0) & (ahead <= self.look_ahead)
        hits = np.where(band & in_range[None, :], ahead[None, :], math.inf)
        free = hits.min(axis=1)
        touching = apart < fp.half_width + ALLOWANCE_M if passing else band
        free[(touching & beside[None, :]).any(axis=1)] = 0.0
        return free

    def bounds(
        self, s_pts, d_pts, s_robot: float, at: float, ahead_gap=None, beside_gap=None
    ) -> tuple[float, float]:
        """Return the offsets (low, high) around at that keep a gap to the nearest points."""
        low, high = -math.inf, math.inf
        if np.size(s_pts) == 0:
            return low, high
        fp = self.footprint
        ahead = s_pts - (s_robot + fp.front)
        beside = (s_pts >= s_robot - fp.rear) & (ahead <= 0.0)
        in_range = (ahead > 0.0) & (ahead <= self.look_ahead)
        for mask, gap in ((in_range, ahead_gap), (beside, beside_gap)):
            if gap is None:
                continue
            d = d_pts[mask]
            right, left = d[d < at], d[d >= at]
            if right.size:
                low = max(low, float(right.max()) + gap)
            if left.size:
                high = min(high, float(left.min()) - gap)
        return low, high
