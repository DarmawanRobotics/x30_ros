"""Correlative scan-to-map matching on a likelihood field (Olson 2009), coarse to fine."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.grid.distance import distance_field, likelihood_field
from x30_utils.grid.spec import GridSpec
from x30_utils.math.se2 import Pose2D


@dataclass(frozen=True)
class MatchConfig:
    """Search window and scoring."""

    xy_window: float = 0.3
    yaw_window: float = math.radians(6.0)
    xy_step: float = 0.05
    yaw_step: float = math.radians(1.0)
    sigma: float = 0.1
    max_points: int = 300
    min_score: float = 0.5


@dataclass(frozen=True)
class MatchResult:
    """Best pose, mean likelihood of the points (0..1) and the covariance of the estimate."""

    pose: Pose2D
    score: float
    covariance: np.ndarray

    @property
    def ok(self) -> bool:
        """True when the covariance is usable."""
        return bool(np.all(np.isfinite(self.covariance)))


class ScanMatcher:
    """Align 2D points (robot frame) with an occupancy grid around a prior pose."""

    def __init__(self, occupied: np.ndarray, spec: GridSpec, config: MatchConfig | None = None):
        self.spec = spec
        self.config = config or MatchConfig()
        distance = distance_field(occupied, spec.resolution, 4.0 * self.config.sigma)
        self.field = likelihood_field(distance, self.config.sigma)

    def score(self, points_map: np.ndarray) -> float:
        """Return the mean likelihood of points already in the map frame."""
        return float(self._scores(np.asarray(points_map, dtype=float)[None])[0])

    def _scores(self, batch: np.ndarray) -> np.ndarray:
        """Return the mean likelihood of each (M, N, 2) set of map-frame points."""
        m, n = batch.shape[:2]
        if n == 0:
            return np.zeros(m)
        cells = self.spec.cells(batch.reshape(-1, 2))
        inside = self.spec.inside(cells)
        values = np.zeros(len(cells), dtype=np.float32)
        values[inside] = self.field[cells[inside, 1], cells[inside, 0]]
        return values.reshape(m, n).mean(axis=1)

    def match(self, points: np.ndarray, prior: Pose2D) -> MatchResult | None:
        """Return the best pose within the window, or None below min_score."""
        cfg = self.config
        points = self._subsample(np.asarray(points, dtype=float).reshape(-1, 2))
        if len(points) < 10:
            return None
        best, poses, scores = self._search(points, prior, cfg.xy_window, cfg.yaw_window, 1.0)
        best, poses_f, scores_f = self._search(points, best, cfg.xy_step, cfg.yaw_step, 0.25)
        score = self.score(best.apply(points))
        if score < cfg.min_score:
            return None
        poses = np.vstack((poses, poses_f))
        scores = np.concatenate((scores, scores_f))
        return MatchResult(best, score, _covariance(poses, scores, best))

    def _search(self, points, center: Pose2D, xy_win: float, yaw_win: float, scale: float):
        cfg = self.config
        xy_step, yaw_step = cfg.xy_step * scale, cfg.yaw_step * scale
        steps = np.arange(-xy_win, xy_win + 1e-9, xy_step)
        grid_x, grid_y = np.meshgrid(steps, steps, indexing='ij')
        shifts = np.column_stack((grid_x.ravel() + center.x, grid_y.ravel() + center.y))
        poses, scores = [], []
        for dyaw in np.arange(-yaw_win, yaw_win + 1e-9, yaw_step):
            yaw = center.yaw + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            rotated = points @ np.array([[c, s], [-s, c]])
            scores.append(self._scores(rotated[None, :, :] + shifts[:, None, :]))
            poses.append(np.column_stack((shifts, np.full(len(shifts), yaw))))
        poses, scores = np.vstack(poses), np.concatenate(scores)
        x, y, yaw = poses[int(np.argmax(scores))]
        return Pose2D(float(x), float(y), float(yaw)), poses, scores

    def _subsample(self, points: np.ndarray) -> np.ndarray:
        n = self.config.max_points
        if len(points) <= n:
            return points
        return points[np.linspace(0, len(points) - 1, n).astype(int)]


def _covariance(poses: np.ndarray, scores: np.ndarray, best: Pose2D) -> np.ndarray:
    """Covariance of the pose from the score surface, weights exp(k * normalized score)."""
    weights = np.exp(20.0 * (scores - scores.max()))
    weights /= weights.sum()
    d = poses - (best.x, best.y, best.yaw)
    d[:, 2] = (d[:, 2] + math.pi) % (2.0 * math.pi) - math.pi
    cov = (d * weights[:, None]).T @ d
    floor = np.diag([1e-4, 1e-4, math.radians(0.5) ** 2])
    return cov + floor
