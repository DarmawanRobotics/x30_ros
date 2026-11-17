"""Blind LiDAR detection: too few points overall or in part of the field of view."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Blindness:
    """Whether a cloud is too sparse to trust, and why."""

    blind: bool
    reason: str = ''


def check_blind(
    xyz: np.ndarray, min_points: int, regions: int = 1, min_region_points: int = 0
) -> Blindness:
    """Check a cloud (sensor frame): min_points in total, min_region_points per azimuth region."""
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if len(xyz) < min_points:
        return Blindness(True, f'{len(xyz)} points (min {min_points})')
    if min_region_points <= 0 or regions <= 1:
        if len(xyz) < min_region_points:
            return Blindness(True, f'{len(xyz)} points (min {min_region_points})')
        return Blindness(False)
    azimuth = np.mod(np.arctan2(xyz[:, 1], xyz[:, 0]) + math.pi, 2.0 * math.pi)
    index = np.minimum((azimuth / (2.0 * math.pi / regions)).astype(np.int64), regions - 1)
    counts = np.bincount(index, minlength=regions)
    worst = int(np.argmin(counts))
    if counts[worst] < min_region_points:
        start = math.degrees(-math.pi + worst * 2.0 * math.pi / regions)
        return Blindness(True, f'{counts[worst]} points in region {worst} (from {start:.0f} deg)')
    return Blindness(False)
