"""Sector scans without ROS; a NaN, non-finite or BLOCKED range is an obstacle at the robot."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.robot import Footprint
from x30_utils.safety import BLOCKED_RANGE, SectorPoints, sector_points


@dataclass(frozen=True, eq=False)
class SectorScanData:
    """Nearest obstacle per sector, robot frame; sector i is centred at min + (i + 0.5) inc."""

    angle_min: float
    angle_increment: float
    range_max: float
    ranges: np.ndarray

    @staticmethod
    def of(angle_min: float, angle_increment: float, range_max: float, ranges) -> SectorScanData:
        """Build from message fields."""
        return SectorScanData(
            float(angle_min), float(angle_increment), float(range_max), np.asarray(ranges, float)
        )

    @property
    def angles(self) -> np.ndarray:
        """Centre angle of every sector (rad)."""
        return self.angle_min + (np.arange(len(self.ranges)) + 0.5) * self.angle_increment

    def points(self, edges: bool = True) -> SectorPoints:
        """Return the obstacle points (edges: at both sector edges too) and blocked sectors."""
        args = (self.angle_increment, self.ranges, self.range_max)
        centre = sector_points(self.angle_min, *args)
        if not edges:
            return centre
        half = 0.5 * self.angle_increment
        lo = sector_points(self.angle_min - half, *args)
        hi = sector_points(self.angle_min + half, *args)
        return SectorPoints(
            np.vstack((centre.points, lo.points, hi.points)), centre.blocked, centre.full_circle
        )

    def cleaned(self) -> np.ndarray:
        """Return the ranges with blocked or unknown sectors at 0 and the rest <= range_max."""
        r = self.ranges
        with np.errstate(invalid='ignore'):  # NaN is blocked (numpy 1.14 warns)
            blocked = ~np.isfinite(r) | (r <= BLOCKED_RANGE)
        return np.where(blocked, 0.0, np.minimum(np.nan_to_num(r), self.range_max))


def body_extent(footprint: Footprint, angles: np.ndarray) -> np.ndarray:
    """Return the distance from base_link to the footprint edge along every angle."""
    c, s = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore'):
        ahead = np.where(c < 0.0, -footprint.rear / c, math.inf)
        along = np.where(c > 0.0, footprint.front / c, ahead)
        across = np.where(np.abs(s) > 1e-12, footprint.half_width / np.abs(s), math.inf)
    return np.minimum(along, across)
