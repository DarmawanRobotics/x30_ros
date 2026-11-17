from __future__ import annotations

import math

import numpy as np


def finite(*values: float) -> bool:
    """Return True when every value is a finite number."""
    try:
        return all(math.isfinite(v) for v in values)
    except TypeError:
        return False


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a value; a non-finite value becomes 0 before clamping (safe for commands)."""
    if not finite(value):
        value = 0.0
    return min(high, max(low, value))


def clamp_abs(value: float, limit: float) -> float:
    """Clamp a value to [-limit, limit]."""
    return clamp(value, -abs(limit), abs(limit))


def arc_lengths(points: np.ndarray) -> np.ndarray:
    """Return the cumulative length along an (N, 2) polyline, starting at 0."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return np.zeros(0)
    steps = np.hypot(*np.diff(points, axis=0).T)
    return np.concatenate(([0.0], np.cumsum(steps)))


def point_segment_distance(p, a, b) -> float:
    """Return the distance from point p to segment ab."""
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, float((p - a) @ ab) / denom))
    return float(np.linalg.norm(p - (a + t * ab)))


def polar_to_xy(angles: np.ndarray, ranges: np.ndarray) -> np.ndarray:
    """Return (N, 2) points from angles (rad) and ranges (m)."""
    angles, ranges = np.asarray(angles, dtype=float), np.asarray(ranges, dtype=float)
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
