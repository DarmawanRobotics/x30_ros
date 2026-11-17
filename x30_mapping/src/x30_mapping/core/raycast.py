"""2D free-space rays from the sensor to the nearest hit per bin (no hit: nothing carved)."""

from __future__ import annotations

import math

import numpy as np

from x30_utils.grid import GridSpec


def nearest_per_bin(dx: np.ndarray, dy: np.ndarray, bins: int) -> tuple[np.ndarray, np.ndarray]:
    """Return (angle, distance) of the nearest point of every occupied angular bin."""
    dist = np.hypot(dx, dy)
    angle = np.arctan2(dy, dx)
    bins = max(1, int(bins))
    b = np.floor((angle + math.pi) / (2.0 * math.pi) * bins).astype(np.int64) % bins
    order = np.lexsort((dist, b))
    first = np.ones(order.size, dtype=bool)
    first[1:] = b[order][1:] != b[order][:-1]
    nearest = order[first]
    return angle[nearest], dist[nearest]


def ray_cells(
    spec: GridSpec, sx: float, sy: float, angles: np.ndarray, reach: np.ndarray
) -> np.ndarray:
    """Return the unique flat cells (row * width + col) crossed by rays up to reach."""
    reach = np.asarray(reach, dtype=np.float64)
    ok = reach > 0.0
    if not ok.any():
        return np.empty(0, dtype=np.int64)
    angles, reach = np.asarray(angles)[ok], reach[ok]
    step = spec.resolution * 0.5
    t = np.arange(math.ceil(reach.max() / step) + 1, dtype=np.float64) * step
    inside = t[None, :] <= reach[:, None]
    xs = sx + np.cos(angles)[:, None] * t[None, :]
    ys = sy + np.sin(angles)[:, None] * t[None, :]
    cells = spec.cells(np.column_stack((xs[inside], ys[inside])))
    cells = cells[spec.inside(cells)]
    return np.unique(cells[:, 1] * spec.width + cells[:, 0])
