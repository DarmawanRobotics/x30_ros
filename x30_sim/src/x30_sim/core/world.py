"""The 2.5D world in the map frame: walls, pillars and dynamic obstacles with heights."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
import yaml

from x30_utils.math import Pose2D, quat_from_rpy

WALL_HEIGHT = 2.5  # m, default for walls without a height
PILLAR_HEIGHT = 3.0  # m, default for pillars without a height


class WorldError(ValueError):
    """The world file cannot be used."""


@dataclass(frozen=True)
class World:
    """Static walls (N, 5: x0, y0, x1, y1, height) and circles (N, 4: x, y, radius, height)."""

    segments: np.ndarray = field(default_factory=lambda: np.empty((0, 5)))
    circles: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    start: Pose2D = field(default_factory=Pose2D)
    tags: dict = field(default_factory=dict)  # frame -> (x, y, z, qx, qy, qz, qw), map frame
    dock: dict = field(default_factory=dict)  # {tag, distance, tolerance, yaw_tolerance_deg}

    @staticmethod
    def from_yaml(path: str) -> World:
        """Load a world file (see config/worlds/mall_loop.yaml); raise WorldError."""
        try:
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as error:
            raise WorldError(f'{path}: {error}') from error
        try:
            return World.from_dict(data)
        except (TypeError, ValueError, KeyError) as error:
            raise WorldError(f'{path}: {error}') from error

    @staticmethod
    def from_dict(data: dict) -> World:
        """Build a world from walls (polylines), pillars, start [x, y, yaw_deg], tags and dock."""
        segments = []
        for wall in data.get('walls', []):
            points, height = _wall(wall)
            segments += [(*a, *b, height) for a, b in zip(points, points[1:])]
        circles = [_circle(c, PILLAR_HEIGHT) for c in data.get('pillars', [])]
        x, y, yaw_deg = (float(v) for v in data.get('start', [0.0, 0.0, 0.0]))
        tags = {str(k): _tag(v) for k, v in (data.get('tags') or {}).items()}
        world = World(
            np.array(segments, dtype=float).reshape(-1, 5),
            np.array(circles, dtype=float).reshape(-1, 4),
            Pose2D(x, y, math.radians(yaw_deg)),
            tags,
            dict(data.get('dock') or {}),
        )
        if not (np.all(np.isfinite(world.segments)) and np.all(np.isfinite(world.circles))):
            raise ValueError('non-finite coordinates')
        return world

    def with_circles(self, extra: np.ndarray) -> np.ndarray:
        """Return the static circles plus extra (N, 4) circles."""
        extra = np.asarray(extra, dtype=float).reshape(-1, 4)
        return np.vstack([self.circles, extra]) if len(extra) else self.circles


def _wall(wall) -> tuple[list, float]:
    """Return the points and height of a wall: a polyline or {points: [...], height: h}."""
    if isinstance(wall, dict):
        points, height = wall['points'], float(wall.get('height', WALL_HEIGHT))
    else:
        points, height = wall, WALL_HEIGHT
    points = [(float(p[0]), float(p[1])) for p in points]
    if len(points) < 2 or height <= 0.0:
        raise ValueError('a wall needs two points and a positive height')
    return points, height


def _circle(values, default_height: float) -> tuple:
    """Return (x, y, radius, height) of [x, y, radius] or [x, y, radius, height]."""
    values = [float(v) for v in values]
    if len(values) not in (3, 4) or values[2] <= 0.0:
        raise ValueError(f'a circle is [x, y, radius(, height)], got {values}')
    return (*values[:3], values[3] if len(values) == 4 else default_height)


def _tag(values) -> tuple:
    """Return the tag pose (x, y, z, qx, qy, qz, qw) of a wall plate [x, y, z, facing_deg]."""
    x, y, z, facing_deg = (float(v) for v in values)
    q = quat_from_rpy(-math.pi / 2.0, 0.0, math.radians(facing_deg) - math.pi / 2.0)
    return (x, y, z, *(float(v) for v in q))


def segment_distances(segments: np.ndarray, ox: float, oy: float, angles: np.ndarray):
    """Return (A, K) distances along each ray to each segment (inf: missed)."""
    angles = np.asarray(angles, dtype=float)
    if not len(segments):
        return np.empty((len(angles), 0))
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    x0, y0, x1, y1 = (segments[:, i][None, :] for i in range(4))
    ex, ey = x1 - x0, y1 - y0
    denom = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((x0 - ox) * ey - (y0 - oy) * ex) / denom
        u = ((x0 - ox) * dy - (y0 - oy) * dx) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    return np.where(hit, t, np.inf)


def circle_distances(circles: np.ndarray, ox: float, oy: float, angles: np.ndarray):
    """Return (A, K) distances along each ray to each circle (inf: missed; inside: far side)."""
    angles = np.asarray(angles, dtype=float)
    if not len(circles):
        return np.empty((len(angles), 0))
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    cx, cy, r = (circles[:, i][None, :] for i in range(3))
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    root = np.sqrt(np.maximum(disc, 0.0))
    t = np.where(c < 0.0, -b + root, -b - root)
    hit = (disc >= 0.0) & (t > 0.0)
    return np.where(hit, t, np.inf)


def ray_distances(world: World, ox: float, oy: float, angles: np.ndarray, extra=None):
    """Return ((A, K) distances to every object, (K,) object heights) for rays from (ox, oy)."""
    circles = world.with_circles(extra if extra is not None else np.empty((0, 4)))
    dist = np.hstack(
        [
            segment_distances(world.segments, ox, oy, angles),
            circle_distances(circles, ox, oy, angles),
        ]
    )
    heights = np.concatenate([world.segments[:, 4], circles[:, 3]])
    return dist, heights


def cast_rays(world: World, ox: float, oy: float, angles, max_range: float, extra=None):
    """Return the horizontal distance to the nearest object along each ray (max_range: none)."""
    dist, _ = ray_distances(world, ox, oy, angles, extra)
    nearest = dist.min(axis=1) if dist.shape[1] else np.full(len(dist), np.inf)
    return np.minimum(nearest, max_range)


def _box_hits_segment(p0: np.ndarray, p1: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> bool:
    """Return True when the segment p0-p1 touches the axis-aligned box lo..hi (Liang-Barsky)."""
    t0, t1 = 0.0, 1.0
    d = p1 - p0
    for axis in range(2):
        if abs(d[axis]) < 1e-12:
            if p0[axis] < lo[axis] or p0[axis] > hi[axis]:
                return False
            continue
        a, b = (lo[axis] - p0[axis]) / d[axis], (hi[axis] - p0[axis]) / d[axis]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def collides(world: World, pose: Pose2D, lo, hi, extra=None) -> bool:
    """Return True when the box lo..hi (base frame, e.g. the footprint) touches any object."""
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    inv = pose.inverse()
    for seg in world.segments:
        a, b = inv.apply(np.array([seg[0:2], seg[2:4]]))
        if _box_hits_segment(a, b, lo, hi):
            return True
    circles = world.with_circles(extra if extra is not None else np.empty((0, 4)))
    if len(circles):
        centers = inv.apply(circles[:, :2])
        nearest = np.clip(centers, lo, hi)
        if np.any(np.hypot(*(centers - nearest).T) <= circles[:, 2]):
            return True
    return False
