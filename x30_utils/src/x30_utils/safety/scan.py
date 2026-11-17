"""Sector scans (x30_msgs/SectorScan) as obstacle points, blocked sectors included."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.math import polar_to_xy

BLOCKED_RANGE = 1e-3


@dataclass(frozen=True)
class SectorPoints:
    """Obstacle points of a sector scan (robot frame) and which sectors are blocked."""

    points: np.ndarray
    blocked: np.ndarray
    full_circle: bool

    @property
    def any_blocked(self) -> bool:
        """True when a sector is blocked or unknown (blind sensor)."""
        return bool(self.blocked.any())

    def with_blocked(self) -> np.ndarray:
        """Points plus one point at the robot for every blocked sector (fail-safe input)."""
        if not self.any_blocked:
            return self.points
        return np.vstack((self.points, np.zeros((int(self.blocked.sum()), 2))))


def sector_points(
    angle_min: float, angle_increment: float, ranges, range_max: float
) -> SectorPoints:
    """Convert sector ranges: >= range_max is free, <= 0 or NaN is blocked."""
    r = np.asarray(ranges, dtype=float)
    with np.errstate(invalid='ignore'):  # NaN compares False; numpy < 1.16 also warns
        blocked = ~np.isfinite(r) | (r <= BLOCKED_RANGE)
        hits = ~blocked & (r < range_max - 1e-3)
    theta = angle_min + (np.arange(len(r)) + 0.5) * angle_increment
    step = abs(angle_increment)
    full = len(r) > 0 and abs(step * len(r) - 2.0 * math.pi) <= step
    return SectorPoints(polar_to_xy(theta[hits], r[hits]), blocked, full)
