from __future__ import annotations

import math

import numpy as np

from x30_utils.grid import GridSpec, MatchConfig, ScanMatcher
from x30_utils.math import Pose2D


def room(spec: GridSpec) -> np.ndarray:
    grid = np.zeros((spec.height, spec.width), dtype=bool)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True
    grid[40:60, 30] = True
    return grid


def test_cells_floor_negative_coordinates():
    spec = GridSpec(0.1, -1.0, -1.0, 20, 20)
    assert spec.cells([[-0.95, -0.05]]).tolist() == [[0, 9]]
    assert spec.cells([[-1.01, 0.0]]).tolist() == [[-1, 10]]
    assert not spec.inside(spec.cells([[-1.01, 0.0]]))[0]
    assert np.allclose(spec.centers([[0, 0]]), [[-0.95, -0.95]])


def test_scan_matcher_recovers_an_offset():
    spec = GridSpec(0.05, 0.0, 0.0, 100, 100)
    occupied = room(spec)
    truth = Pose2D(2.5, 2.4, math.radians(10))
    walls = spec.centers(np.argwhere(occupied)[:, ::-1])
    points = truth.inverse().apply(walls)
    points = points[np.hypot(points[:, 0], points[:, 1]) < 4.0]
    matcher = ScanMatcher(occupied, spec, MatchConfig(xy_window=0.25, yaw_window=math.radians(5)))
    result = matcher.match(points, Pose2D(2.62, 2.30, math.radians(7)))
    assert result is not None and result.ok
    assert result.pose.distance(truth) < 0.04
    assert abs(result.pose.yaw - truth.yaw) < math.radians(1.0)
    assert result.score > 0.8
