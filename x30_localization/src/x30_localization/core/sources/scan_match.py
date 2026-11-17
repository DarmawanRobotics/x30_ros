"""Scan-to-map source: the registered body cloud projected to 2D and matched against /map."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_localization.core.sources.base import Measurement, Source
from x30_utils.grid import GridSpec, MatchConfig, ScanMatcher
from x30_utils.math import Pose2D, transform_points


@dataclass(frozen=True)
class ScanBand:
    """Which points of the cloud are used, relative to the floor (m)."""

    min_height: float = 0.1
    max_height: float = 1.5
    max_range: float = 15.0


def planar_points(
    points_body: np.ndarray, m_odom_body: np.ndarray, m_odom_base: np.ndarray,
    base_height: float, band: ScanBand,
) -> np.ndarray:  # fmt: skip
    """Return (N, 2) planar base points, the height band taken in the level odom frame."""
    points = transform_points(m_odom_body, np.asarray(points_body, dtype=float).reshape(-1, 3))
    floor = m_odom_base[2, 3] - base_height
    base = Pose2D.from_matrix(m_odom_base)
    xy = (points[:, :2] - (base.x, base.y)) @ np.array(
        [[np.cos(base.yaw), -np.sin(base.yaw)], [np.sin(base.yaw), np.cos(base.yaw)]]
    )
    height = points[:, 2] - floor
    reach = np.hypot(xy[:, 0], xy[:, 1])
    keep = (height >= band.min_height) & (height <= band.max_height) & (reach <= band.max_range)
    return xy[keep]


class ScanMatchSource(Source):
    """Measurements of map->base from correlative matching against an occupancy grid."""

    name = 'scan_match'

    def __init__(self, config: MatchConfig | None = None, covariance_scale: float = 4.0):
        self.config = config or MatchConfig()
        self.covariance_scale = covariance_scale
        self.matcher: ScanMatcher | None = None

    def set_map(self, occupied: np.ndarray, spec: GridSpec):
        """Use a new occupancy grid (boolean, rows = y)."""
        self.matcher = ScanMatcher(np.asarray(occupied, dtype=bool), spec, self.config)

    def measure(self, points_base: np.ndarray, prior: Pose2D, stamp: float) -> Measurement | str:
        """Return the measurement of matching base-frame points around prior, or why not."""
        if self.matcher is None:
            return 'no map'
        result = self.matcher.match(points_base, prior)
        if result is None:
            return 'no match above min_score'
        if not result.ok:
            return 'match covariance unusable'
        cov = np.asarray(result.covariance, dtype=float) * self.covariance_scale
        return Measurement(result.pose, cov, stamp, self.name)


def match_config(xy_window: float, yaw_window_deg: float, min_score: float) -> MatchConfig:
    """Return the matcher configuration of the node parameters."""
    return MatchConfig(
        xy_window=xy_window, yaw_window=math.radians(yaw_window_deg), min_score=min_score
    )
