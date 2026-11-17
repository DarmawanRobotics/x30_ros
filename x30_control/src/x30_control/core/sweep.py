"""Free distance of the footprint along a straight translation (the swept footprint)."""

# Kept identical in x30_navigation and x30_control: the guard and the field share it.

from __future__ import annotations

import math

import numpy as np

from x30_utils.robot import Footprint

ALLOWANCE_M = 0.02
EPS = 1e-12


def box_distance(footprint: Footprint, points: np.ndarray) -> np.ndarray:
    """Return the distance (m) of every point (robot frame) from the footprint (0: inside)."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    ex = np.maximum(np.maximum(p[:, 0] - footprint.front, -footprint.rear - p[:, 0]), 0.0)
    ey = np.maximum(np.abs(p[:, 1]) - footprint.half_width, 0.0)
    return np.hypot(ex, ey)


def entry_distances(
    footprint: Footprint, ux: float, uy: float, points: np.ndarray, margin
) -> np.ndarray:
    """Return how far the footprint widened across u by margin moves along u until each point."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    m = np.broadcast_to(np.asarray(margin, dtype=float), (len(p),))
    fp = footprint
    lead = (fp.front if ux >= 0.0 else fp.rear) * abs(ux) + fp.half_width * abs(uy)
    back = (fp.rear if ux >= 0.0 else fp.front) * abs(ux) + fp.half_width * abs(uy)
    normals = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0], [ux, uy], [-ux, -uy]])
    grow_x, grow_y = m * abs(uy), m * abs(ux)  # |n . e| for the across direction n = (-uy, ux)
    lead, back = np.full(len(p), lead), np.full(len(p), back)
    support = np.stack((fp.front + grow_x, fp.rear + grow_x, fp.half_width + grow_y,
                        fp.half_width + grow_y, lead, back))  # fmt: skip
    return _convex_entry(normals, support, p, ux, uy)


def rounded_entry_distances(
    footprint: Footprint, ux: float, uy: float, points: np.ndarray, radius: np.ndarray
) -> np.ndarray:
    """Return how far the footprint moves along u until each point is radius from it."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    r = np.broadcast_to(np.asarray(radius, dtype=float), (len(p),))
    fp, zero = footprint, np.zeros(len(p))
    normals = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
    best = np.full(len(p), math.inf)
    for grow_x, grow_y in ((r, zero), (zero, r)):  # the footprint grown along x or along y
        support = np.stack((fp.front + grow_x, fp.rear + grow_x, fp.half_width + grow_y,
                            fp.half_width + grow_y))  # fmt: skip
        best = np.minimum(best, _convex_entry(normals, support, p, ux, uy))
    for corner in footprint.corners():  # and a disc of radius r at every corner
        q = p - corner
        b = q[:, 0] * ux + q[:, 1] * uy
        disc = b * b - (q[:, 0] ** 2 + q[:, 1] ** 2 - r * r)
        with np.errstate(invalid='ignore'):
            t = np.where((disc >= 0.0) & (b >= 0.0), b - np.sqrt(np.maximum(disc, 0.0)), math.inf)
        best = np.minimum(best, np.maximum(t, 0.0))
    return best


def _convex_entry(normals, support, p: np.ndarray, ux: float, uy: float) -> np.ndarray:
    """Return where the ray p - t u (t >= 0) enters the polygon normals . q <= support."""
    k = normals @ np.array([ux, uy])
    g = normals @ p.T - support  # (faces, N): inside a face when <= 0
    with np.errstate(divide='ignore', invalid='ignore'):
        bound = g / k[:, None]
    entering, leaving, parallel = k > EPS, k < -EPS, np.abs(k) <= EPS
    t_in = np.full(len(p), 0.0)
    if entering.any():
        t_in = np.maximum(bound[entering].max(axis=0), 0.0)
    t_out = bound[leaving].min(axis=0) if leaving.any() else np.full(len(p), math.inf)
    hit = t_in <= t_out
    if parallel.any():
        hit &= np.all(g[parallel] <= 0.0, axis=0)
    return np.where(hit, t_in, math.inf)


def swept_clearance(
    footprint: Footprint,
    ux: float,
    uy: float,
    points: np.ndarray,
    lateral_margin: float = 0.0,
    allowance: float = ALLOWANCE_M,
) -> float:
    """Return the free distance (m) of the footprint along the unit direction (ux, uy)."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if p.size == 0:
        return math.inf
    start = box_distance(footprint, p)
    if np.any(start <= 0.0):
        return 0.0
    free = entry_distances(footprint, ux, uy, p, lateral_margin)
    near = free <= 0.0
    if near.any():
        approach = np.maximum(np.minimum(start[near], allowance) - 1e-3, 0.0)
        free[near] = rounded_entry_distances(footprint, ux, uy, p[near], approach)
    return float(free.min())
