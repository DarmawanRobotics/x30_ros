"""Robust angular binning: the k-th nearest point per sector (REP-103, +CCW from x)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

BLOCKED = 0.0
FULL_CIRCLE = 2.0 * math.pi


@dataclass(frozen=True)
class SectorLayout:
    """Angular sectors: angle_min, fov (rad), count, and the range limits (m)."""

    angle_min: float
    fov: float
    num_sectors: int
    range_min: float
    range_max: float

    @staticmethod
    def front(fov: float, num_sectors: int, range_min: float, range_max: float):
        """Sectors centred ahead over fov."""
        return SectorLayout(-fov / 2.0, fov, num_sectors, range_min, range_max)

    @staticmethod
    def circle(num_sectors: int, range_min: float, range_max: float):
        """Sectors over the full circle, starting behind the robot (-pi)."""
        return SectorLayout(-math.pi, FULL_CIRCLE, num_sectors, range_min, range_max)

    @property
    def increment(self) -> float:
        """Width of one sector (rad)."""
        return self.fov / self.num_sectors

    @property
    def full_circle(self) -> bool:
        """True when the sectors cover 360 deg."""
        return self.fov >= FULL_CIRCLE - 1e-6

    def free(self) -> np.ndarray:
        """Ranges of an empty scan (every sector free)."""
        return np.full(self.num_sectors, self.range_max, dtype=np.float32)

    def blocked(self) -> np.ndarray:
        """Ranges of a blind scan (every sector BLOCKED)."""
        return np.full(self.num_sectors, BLOCKED, dtype=np.float32)

    def locate(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (input rows, sector index, range) of the points inside sectors and limits."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        r = np.hypot(xy[:, 0], xy[:, 1])
        # counter-clockwise from angle_min in [0, 2 pi): any angle_min, any fov up to 360 deg
        rel = np.mod(np.arctan2(xy[:, 1], xy[:, 0]) - self.angle_min, FULL_CIRCLE)
        valid = (rel >= 0.0) & (rel < self.fov) & (r >= self.range_min) & (r <= self.range_max)
        idx = np.minimum((rel[valid] / self.increment).astype(np.int64), self.num_sectors - 1)
        return np.flatnonzero(valid), idx, r[valid]


def kth_ranges(layout: SectorLayout, xy: np.ndarray, k: int = 1) -> np.ndarray:
    """Return the k-th nearest range per sector; range_max with fewer than k points."""
    ranges = layout.free()
    _, idx, r = layout.locate(xy)
    if r.size < k:
        return ranges
    order = np.lexsort((r, idx))
    idx, r = idx[order], r[order]
    counts = np.bincount(idx, minlength=layout.num_sectors)
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    hit = counts >= k
    ranges[hit] = r[starts[hit] + k - 1].astype(np.float32)
    return ranges
