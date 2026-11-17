"""Free space around a moving footprint: along a translation and around a rotation."""

from __future__ import annotations

import math

import numpy as np

from x30_utils.robot import Footprint


def translation_clearance(
    footprint: Footprint, ux: float, uy: float, points: np.ndarray, lateral_margin: float = 0.0
) -> float:
    """Return how far the footprint moves along (ux, uy) before touching a point."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if p.size == 0:
        return math.inf
    along = p[:, 0] * ux + p[:, 1] * uy
    across = np.abs(-p[:, 0] * uy + p[:, 1] * ux)
    edge, half = _extent(footprint, ux, uy)
    ahead = (along >= edge) & (across <= half + lateral_margin)
    beside = (along > 0.0) & (along < edge) & (across <= half)
    inside = footprint.contains(p)
    candidates = along[ahead] - edge
    if np.any(beside | inside):
        candidates = np.concatenate((candidates, [0.0]))
    return float(candidates.min()) if candidates.size else math.inf


def rotation_room(footprint: Footprint, points: np.ndarray, direction: float) -> float:
    """Return the angle (rad) the footprint turns toward sign(direction) before a point."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if p.size == 0:
        return math.inf
    r = np.hypot(p[:, 0], p[:, 1])
    ring = p[(r > footprint.inscribed_radius) & (r <= footprint.circumscribed_radius)]
    if np.any(footprint.contains(p)):
        return 0.0
    if ring.size == 0:
        return math.inf
    sign = 1.0 if direction >= 0.0 else -1.0
    for step in np.radians(np.arange(1.0, 181.0, 1.0)):
        c, s = math.cos(-sign * step), math.sin(-sign * step)
        rotated = ring @ np.array([[c, s], [-s, c]])
        if np.any(footprint.contains(rotated)):
            return float(step - math.radians(1.0))
    return math.pi


def _extent(footprint: Footprint, ux: float, uy: float) -> tuple[float, float]:
    """Return (distance to the leading edge, half width across) for a motion direction."""
    corners = footprint.corners()
    along = corners @ np.array([ux, uy])
    across = corners @ np.array([-uy, ux])
    return float(along.max()), float(np.abs(across).max())
