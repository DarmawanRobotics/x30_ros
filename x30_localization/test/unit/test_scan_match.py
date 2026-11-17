"""Optional scan-to-map source (L1): projection with a pitched body, match covariance."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_localization.core.sources import sources
from x30_localization.core.sources.scan_match import ScanBand, ScanMatchSource, planar_points
from x30_utils.grid import GridSpec, MatchConfig
from x30_utils.math import (
    Pose2D,
    default_rng,
    inverse,
    make_transform,
    quat_from_rpy,
    transform_points,
)

RES = 0.05
SPEC = GridSpec(RES, -1.0, -1.0, 200, 160)  # 10 m x 8 m


def room() -> np.ndarray:
    """Occupancy of a walled room with an inner box (rows = y)."""
    occupied = np.zeros((SPEC.height, SPEC.width), dtype=bool)
    occupied[0, :] = occupied[-1, :] = True
    occupied[:, 0] = occupied[:, -1] = True
    occupied[60:80, 120:126] = True
    return occupied


def wall_points(occupied: np.ndarray, base: Pose2D, height_m: float) -> np.ndarray:
    """Return 3D points on the occupied cells (in the map), at height_m above the floor."""
    rows, cols = np.nonzero(occupied)
    xy = SPEC.centers(np.column_stack((cols, rows)))
    near = np.hypot(xy[:, 0] - base.x, xy[:, 1] - base.y) < 6.0
    xy = xy[near][::2]
    return np.column_stack((xy, np.full(len(xy), height_m)))


def test_planar_points_level_a_pitched_body_and_drop_the_floor():
    base_height = 0.35
    # base pitched 8 deg nose down by the gait; the LiDAR (body) 0.2 m above, pitched 10 deg
    m_odom_base = make_transform((2.0, 1.0, base_height), quat_from_rpy(0.0, math.radians(8), 0.5))
    m_base_body = make_transform((0.1, 0.0, 0.2), quat_from_rpy(0.0, math.radians(10), 0.0))
    m_odom_body = m_odom_base @ m_base_body
    world = np.array([[4.0, 2.0, 0.5], [3.0, 1.0, 0.0], [3.5, 0.0, 0.02], [5.0, 3.0, 1.0]])
    body = transform_points(inverse(m_odom_body), world)
    xy = planar_points(body, m_odom_body, m_odom_base, base_height, ScanBand(0.1, 1.5, 15.0))
    assert len(xy) == 2  # the two floor points are gone despite the pitch
    expected = Pose2D(2.0, 1.0, 0.5).inverse().apply(world[[0, 3], :2])
    assert xy == pytest.approx(expected)


def test_match_recovers_the_pose_and_reports_a_covariance():
    occupied = room()
    source = sources.create('scan_match', MatchConfig(xy_window=0.3), covariance_scale=4.0)
    assert source.measure(np.zeros((20, 2)), Pose2D(), 0.0) == 'no map'
    source.set_map(occupied, SPEC)
    truth = Pose2D(3.0, 2.5, 0.4)
    points = wall_points(occupied, truth, 0.6)
    base_xy = truth.inverse().apply(points[:, :2])
    m = source.measure(base_xy, Pose2D(3.15, 2.4, 0.45), stamp=5.0)
    assert not isinstance(m, str), m
    assert m.z.to_array() == pytest.approx(truth.to_array(), abs=0.06)
    assert m.valid() and m.source == 'scan_match' and m.stamp == 5.0
    sigma = np.sqrt(np.diag(m.covariance))
    assert np.all(sigma[:2] < 0.2) and sigma[2] < math.radians(5)


def test_match_without_structure_is_refused():
    source = ScanMatchSource(MatchConfig(min_score=0.5))
    source.set_map(room(), SPEC)
    rng = default_rng(2)
    noise = rng.uniform(-0.8, 0.8, (200, 2))  # nothing near any wall
    result = source.measure(noise, Pose2D(4.0, 3.0, 0.0), 0.0)
    assert isinstance(result, str)
