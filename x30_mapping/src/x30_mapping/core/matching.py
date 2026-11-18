"""Scan-to-map checks on a window of the current grid (x30_utils.grid.ScanMatcher)."""

from __future__ import annotations

import math

import numpy as np

from x30_mapping.core.config import GateConfig, RefineConfig
from x30_mapping.core.grid import LogOddsGrid
from x30_utils.grid import GridSpec, MatchConfig, MatchResult, ScanMatcher
from x30_utils.math import Pose2D

SIGMA_M = 0.1  # likelihood field fall-off


def local_matcher(
    grid: LogOddsGrid, center: tuple[float, float], radius: float, config: MatchConfig
) -> tuple[ScanMatcher | None, int]:
    """Return a matcher on the grid window around center and its occupied cell count."""
    spec = grid.spec
    (c0, r0), (c1, r1) = spec.cells(
        np.array(
            [[center[0] - radius, center[1] - radius], [center[0] + radius, center[1] + radius]]
        )
    )
    c0, r0 = max(int(c0), 0), max(int(r0), 0)
    c1, r1 = min(int(c1) + 1, spec.width), min(int(r1) + 1, spec.height)
    if c1 <= c0 or r1 <= r0:
        return None, 0
    occupied = grid.occupied_mask()[r0:r1, c0:c1]
    count = int(np.count_nonzero(occupied))
    if count == 0:
        return None, 0
    window = GridSpec(
        spec.resolution,
        spec.origin_x + c0 * spec.resolution,
        spec.origin_y + r0 * spec.resolution,
        c1 - c0,
        r1 - r0,
    )
    return ScanMatcher(occupied, window, config), count


def agreement(grid: LogOddsGrid, points_map: np.ndarray, center, radius: float) -> float | None:
    """Return the mean likelihood of map-frame points on the grid, None without a map there."""
    matcher, count = local_matcher(grid, center, radius, MatchConfig(sigma=SIGMA_M))
    if matcher is None or count < 10:
        return None
    return matcher.score(points_map)


def refine(
    grid: LogOddsGrid, points_map: np.ndarray, prior: Pose2D, config: RefineConfig, radius: float
) -> tuple[Pose2D | None, str]:
    """Return the matched sensor pose (or None) and why it was not used."""
    match_config = MatchConfig(
        xy_window=config.xy_window,
        yaw_window=math.radians(config.yaw_window_deg),
        xy_step=max(grid.resolution, 0.02),
        yaw_step=math.radians(0.5),
        sigma=SIGMA_M,
        min_score=config.min_score,
    )
    margin = config.xy_window + 4.0 * SIGMA_M
    matcher, count = local_matcher(grid, (prior.x, prior.y), radius + margin, match_config)
    if matcher is None or count < config.min_occupied:
        return None, f'{count} occupied cells near the robot'
    local = prior.inverse().apply(points_map)
    result: MatchResult | None = matcher.match(local, prior)
    if result is None or not result.ok:
        return None, 'no match above min_score'
    sigma = math.sqrt(max(result.covariance[0, 0], result.covariance[1, 1]))
    if sigma > config.max_sigma_m:
        return None, f'ambiguous match (sigma {sigma:.3f} m)'
    return result.pose, ''


def jump_limits(gate: GateConfig, max_speed: float, max_wz: float, dt: float):
    """Return the (distance m, yaw rad) a sensor may move between scans dt apart."""
    dt = min(max(dt, 0.0), 10.0)
    return gate.max_jump_m + max_speed * dt, math.radians(gate.max_jump_deg) + max_wz * dt
