"""Route processing: dedupe, resample and smooth between anchors that never move."""

from __future__ import annotations

import math

import numpy as np

from x30_utils.io import RoutePoint
from x30_utils.math import arc_lengths


def dedupe(xy: np.ndarray, min_spacing: float) -> np.ndarray:
    """Drop points closer than min_spacing to the previous kept one; both ends are kept."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 3:
        return xy.copy()
    kept = [xy[0]]
    for p in xy[1:-1]:
        if math.hypot(*(p - kept[-1])) >= min_spacing:
            kept.append(p)
    if math.hypot(*(xy[-1] - kept[-1])) > 1e-9 or len(kept) == 1:
        kept.append(xy[-1])
    return np.array(kept)


def resample(xy: np.ndarray, spacing: float) -> np.ndarray:
    """Resample a polyline at a uniform spacing of at most spacing, ends kept exactly."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    s = arc_lengths(xy)
    if len(xy) < 2 or spacing <= 0.0 or s[-1] <= 1e-9:
        return xy.copy()
    n = max(1, math.ceil(s[-1] / spacing - 1e-9))
    targets = np.linspace(0.0, s[-1], n + 1)
    out = np.column_stack((np.interp(targets, s, xy[:, 0]), np.interp(targets, s, xy[:, 1])))
    out[0], out[-1] = xy[0], xy[-1]
    return out


def smooth(xy: np.ndarray, weight_data: float, weight_smooth: float) -> np.ndarray:
    """Smooth a polyline with both ends fixed (the gradient smoother's fixed point)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    n = len(xy)
    if n < 3 or weight_smooth <= 0.0:
        return xy.copy()
    from scipy.linalg import solve_banded

    data = weight_data * max(0.0, 1.0 - 2.0 * weight_smooth)
    m = n - 2
    bands = np.zeros((3, m))
    bands[0, 1:] = -weight_smooth
    bands[1, :] = data + 2.0 * weight_smooth
    bands[2, :-1] = -weight_smooth
    rhs = data * xy[1:-1].copy()
    rhs[0] += weight_smooth * xy[0]
    rhs[-1] += weight_smooth * xy[-1]
    out = xy.copy()
    out[1:-1] = solve_banded((1, 1), bands, rhs)
    return out


def headings(xy: np.ndarray) -> np.ndarray:
    """Return the tangent heading (rad) at every point (central differences)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    n = len(xy)
    if n < 2:
        return np.zeros(n)
    ahead = xy[np.minimum(np.arange(n) + 1, n - 1)]
    behind = xy[np.maximum(np.arange(n) - 1, 0)]
    d = ahead - behind
    return np.arctan2(d[:, 1], d[:, 0])


def polyline_distance(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    """Return the distance from every point to the nearest segment of a polyline."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    line = np.asarray(polyline, dtype=float).reshape(-1, 2)
    if len(line) == 1:
        return np.hypot(*(p - line[0]).T)
    a, ab = line[:-1], np.diff(line, axis=0)
    denom = np.maximum((ab * ab).sum(axis=1), 1e-12)
    rel = p[:, None, :] - a[None, :, :]
    t = np.clip((rel * ab[None]).sum(axis=2) / denom, 0.0, 1.0)
    gap = rel - t[..., None] * ab[None]
    return np.hypot(gap[..., 0], gap[..., 1]).min(axis=1)


def anchor_indices(points: list[RoutePoint]) -> list[int]:
    """Return the indices that must not move: both ends and every stop point."""
    last = len(points) - 1
    inner = [i for i, p in enumerate(points) if p.is_stop and 0 < i < last]
    return [0, *inner, last]


def process_route(
    points: list[RoutePoint], spacing: float, weight_data: float, weight_smooth: float
) -> tuple[list[RoutePoint], float]:
    """Resample and smooth a route between its anchors; return it and its largest deviation (m)."""
    if len(points) < 3:
        return list(points), 0.0
    anchors = anchor_indices(points)
    route = [points[0]]
    deviation = 0.0
    for a, b in zip(anchors, anchors[1:]):
        raw = np.array([(p.x, p.y) for p in points[a : b + 1]])
        dense = smooth(resample(dedupe(raw, 0.5 * spacing), spacing), weight_data, weight_smooth)
        inner = dense[1:-1]
        if len(inner):
            deviation = max(deviation, float(polyline_distance(inner, raw).max()))
            yaws = headings(dense)[1:-1]
            if len(yaws) != len(inner):
                raise ValueError(f'{len(yaws)} headings for {len(inner)} points')
            route += [RoutePoint(float(x), float(y), float(t)) for (x, y), t in zip(inner, yaws)]
        route.append(points[b])
    return route, deviation


def process(points: list[RoutePoint], config) -> tuple[list[RoutePoint], float]:
    """Process a route with a ProcessingConfig (spacing and smoothing weights)."""
    return process_route(
        points, config.spacing, config.smooth_weight_data, config.smooth_weight_smooth
    )
