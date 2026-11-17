"""Short obstacle memory in the odom frame for what drops below the sensor's view."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.math import Pose2D


@dataclass(frozen=True)
class NearField:
    """What the lowest beam can see: sensor position (level base frame) and smallest descent."""

    sensor_height: float
    descent: float
    sensor_xy: tuple[float, float] = (0.0, 0.0)

    @staticmethod
    def of_mount(sensor_to_base: np.ndarray, base_height: float, lower_fov: float) -> NearField:
        """Build from the sensor mount and its lowest beam (rad); the worst azimuth counts."""
        m = np.asarray(sensor_to_base, dtype=float)
        azimuth = np.radians(np.arange(0.0, 360.0, 2.0))
        beams = np.column_stack(
            (
                math.cos(lower_fov) * np.cos(azimuth),
                math.cos(lower_fov) * np.sin(azimuth),
                np.full_like(azimuth, math.sin(lower_fov)),
            )
        )
        down = -(beams @ m[:3, :3].T)[:, 2]
        descent = float(np.arcsin(np.clip(down.min(), -1.0, 1.0)))
        return NearField(base_height + float(m[2, 3]), descent, (float(m[0, 3]), float(m[1, 3])))

    def visible_height(self, xy: np.ndarray) -> np.ndarray:
        """Lowest height above the floor the sensor sees at (N, 2) level base points (m)."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        d = np.hypot(xy[:, 0] - self.sensor_xy[0], xy[:, 1] - self.sensor_xy[1])
        if self.descent <= 1e-6:
            return np.full(d.shape, self.sensor_height)
        return self.sensor_height - d * math.tan(self.descent)

    def radius(self, height: float) -> float:
        """Distance within which an object of this height is below the view (inf: always)."""
        if self.descent <= 1e-6:
            return math.inf
        return max(0.0, (self.sensor_height - height) / math.tan(self.descent))


class ObstacleMemory:
    """Obstacle cells in odom: time last seen and the top height of the obstacle in the cell."""

    def __init__(self, memory_sec: float, radius: float, resolution: float, max_cells: int):
        self.memory_sec = memory_sec
        self.radius = radius
        self.resolution = resolution
        self.max_cells = max_cells
        self._cells: dict[tuple[int, int], tuple[float, float]] = {}

    def __len__(self) -> int:
        return len(self._cells)

    def clear(self):
        """Forget everything (odometry restarted or jumped)."""
        self._cells.clear()

    def update(self, odom_base: Pose2D, xy: np.ndarray, height: np.ndarray, now: float):
        """Add obstacle points (level base frame, height above the floor) seen at time now."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        height = np.asarray(height, dtype=float).reshape(-1)
        near = np.hypot(xy[:, 0], xy[:, 1]) <= self.radius
        if near.any():
            keys = np.floor(odom_base.apply(xy[near]) / self.resolution).astype(np.int64)
            unique, inverse = np.unique(keys, axis=0, return_inverse=True)
            top = np.full(len(unique), -np.inf)
            np.maximum.at(top, inverse.reshape(-1), height[near])
            for (i, j), h in zip(unique.tolist(), top.tolist()):  # same length
                self._cells[(i, j)] = (now, h)
        self._expire(now)

    def points(self, odom_base: Pose2D, now: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the remembered cell centres (level base frame) and their heights."""
        self._expire(now)
        if not self._cells:
            return np.empty((0, 2)), np.empty(0)
        keys = np.array(list(self._cells), dtype=float)
        heights = np.array([h for _, h in self._cells.values()])
        xy = odom_base.inverse().apply((keys + 0.5) * self.resolution)
        near = np.hypot(xy[:, 0], xy[:, 1]) <= self.radius
        return xy[near], heights[near]

    def hidden(self, odom_base: Pose2D, now: float, view: NearField) -> np.ndarray:
        """Return the remembered points the sensor cannot see now (top below the lowest beam)."""
        xy, heights = self.points(odom_base, now)
        below = heights < view.visible_height(xy)
        return xy[below]

    def forget(self, odom_base: Pose2D, seen) -> int:
        """Forget the cells a sensor sees now (seen(xy) -> bool per point); return how many."""
        if not self._cells:
            return 0
        keys = list(self._cells)
        xy = odom_base.inverse().apply((np.array(keys, dtype=float) + 0.5) * self.resolution)
        inside = np.asarray(seen(xy), dtype=bool)
        for key in [k for k, s in zip(keys, inside.tolist()) if s]:  # same length
            del self._cells[key]
        return int(inside.sum())

    def _expire(self, now: float):
        oldest = now - self.memory_sec
        stale = [k for k, (t, _) in self._cells.items() if t < oldest or t > now + 1.0]
        for key in stale:
            del self._cells[key]
        if len(self._cells) > self.max_cells:
            newest = sorted(self._cells.items(), key=lambda kv: kv[1][0])[-self.max_cells :]
            self._cells = dict(newest)
