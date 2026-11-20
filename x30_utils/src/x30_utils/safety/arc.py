"""Combined motion: the footprint along the commanded arc until the robot stands still."""

# Shared by the control mux's protective field and the path follower's guard, so the guard is
# never less cautious than the field. A command is anything with vx, vy (m/s) and wz (rad/s).

from __future__ import annotations

import math

import numpy as np

from x30_utils.robot import Footprint, Limits

EPS = 1e-9


def arc_poses(cmd, u: np.ndarray) -> np.ndarray:
    """Return (K, 3) poses (x, y, yaw) reached with the constant twist after each u (s)."""
    u = np.asarray(u, dtype=float)
    theta = cmd.wz * u
    if abs(cmd.wz) <= EPS:
        return np.column_stack((cmd.vx * u, cmd.vy * u, theta))
    sin, one_minus_cos = np.sin(theta), 1.0 - np.cos(theta)
    x = (cmd.vx * sin - cmd.vy * one_minus_cos) / cmd.wz
    y = (cmd.vx * one_minus_cos + cmd.vy * sin) / cmd.wz
    return np.column_stack((x, y, theta))


def box_distances(
    footprint: Footprint, half_width: float, points: np.ndarray, poses: np.ndarray
) -> np.ndarray:
    """Return (K, N) distances from the box (front, rear, half_width) at each pose (0: inside)."""
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    dx = points[None, :, 0] - poses[:, 0, None]
    dy = points[None, :, 1] - poses[:, 1, None]
    bx, by = c * dx + s * dy, -s * dx + c * dy
    ex = np.maximum(np.maximum(bx - footprint.front, -footprint.rear - bx), 0.0)
    ey = np.maximum(np.abs(by) - half_width, 0.0)
    return np.hypot(ex, ey)


class ArcCheck:
    """Largest scale of a translation + rotation command whose stopping arc stays clear."""

    def __init__(
        self,
        footprint: Footprint,
        limits: Limits,
        lateral_margin: float,
        step_m: float = 0.03,
        max_poses: int = 60,
        allowance_m: float = 0.02,
    ):
        self.footprint = footprint
        self.limits = limits
        self.lateral_margin = lateral_margin
        self.step_m = step_m
        self.max_poses = max_poses
        self.allowance_m = allowance_m

    def braking_time(self, cmd) -> float:
        """Time (s) to brake both axes to standstill in proportion."""
        lim = self.limits
        return max(_speed(cmd) / lim.max_decel, abs(cmd.wz) / lim.max_yaw_accel)

    def scale(self, cmd, points: np.ndarray, margin: float, yaw_margin: float) -> float:
        """Return the scale (0..1) for cmd; 1 for a pure translation or rotation."""
        v, w = _speed(cmd), abs(cmd.wz)
        if v <= EPS or w <= EPS:
            return 1.0
        t_r, t_b = self.limits.reaction_time, self.braking_time(cmd)
        extension = min(margin / v, yaw_margin / w)
        u_end = t_r + 0.5 * t_b + extension
        p = self._nearby(points, (v + w * self.footprint.circumscribed_radius) * u_end)
        if p.size == 0:
            return 1.0
        u_clear = self._clear_until(cmd, p, u_end)
        if u_clear >= u_end:
            return 1.0
        room = u_clear - extension
        if room <= 0.0:
            return 0.0
        return min(1.0, (-t_r + math.sqrt(t_r * t_r + 2.0 * t_b * room)) / t_b)

    def _clear_until(self, cmd, points: np.ndarray, u_end: float) -> float:
        """Return the arc parameter of the last clear pose before the first contact."""
        reach = (_speed(cmd) + abs(cmd.wz) * self.footprint.circumscribed_radius) * u_end
        count = int(min(self.max_poses, max(4, math.ceil(reach / self.step_m))))
        us = np.linspace(u_end / count, u_end, count)
        poses = arc_poses(cmd, us)
        hits = self._hits(points, poses)
        if not hits.any():
            return u_end
        first = int(np.argmax(hits))
        return float(us[first - 1]) if first > 0 else 0.0

    def _hits(self, points: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """Return a (K,) mask of the poses at which an obstacle is touched."""
        fp, wide = self.footprint, self.footprint.half_width + self.lateral_margin
        origin = np.zeros((1, 3))
        beside = box_distances(fp, wide, points, origin)[0] <= 0.0
        hits = np.zeros(len(poses), dtype=bool)
        far = points[~beside]
        if far.size:
            hits |= np.any(box_distances(fp, wide, far, poses) <= 0.0, axis=1)
        near = points[beside]
        if near.size:
            start = box_distances(fp, fp.half_width, near, origin)[0]
            allowed = np.minimum(start, self.allowance_m) - 1e-3
            hits |= np.any(box_distances(fp, fp.half_width, near, poses) < allowed, axis=1)
        return hits

    def _nearby(self, points: np.ndarray, reach: float) -> np.ndarray:
        p = np.asarray(points, dtype=float).reshape(-1, 2)
        if p.size == 0:
            return p
        radius = self.footprint.circumscribed_radius + self.lateral_margin + reach
        return p[np.hypot(p[:, 0], p[:, 1]) <= radius]


def _speed(cmd) -> float:
    """Return the linear speed (m/s) of a command."""
    return math.hypot(cmd.vx, cmd.vy)
