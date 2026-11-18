from __future__ import annotations

import math

import numpy as np
import pytest

from x30_perception.core.sectors import BLOCKED, SectorLayout, kth_ranges
from x30_utils.math import default_rng

FRONT = SectorLayout.front(math.pi, 4, 0.1, 8.0)
CIRCLE = SectorLayout.circle(8, 0.1, 4.0)


def test_front_scan_keeps_nearest_per_sector():
    """Front fov: nearest range wins, points outside the fov are ignored."""
    xy = np.array([[2.0, 0.0], [1.5, 0.01], [-3.0, 0.0], [0.0, 2.0]])
    ranges = kth_ranges(FRONT, xy, 1)
    # sectors: [-90,-45) [-45,0) [0,45) [45,90); (0, 2) is exactly at +90 -> outside
    assert ranges[2] == np.float32(np.hypot(1.5, 0.01))
    assert ranges[0] == ranges[1] == ranges[3] == 8.0
    assert ranges.dtype == np.float32


def test_full_circle_wraps_behind():
    """The 360 deg scan sees obstacles behind the robot, at +-180 deg too."""
    xy = np.array([[-1.0, 0.001], [-1.2, -0.001], [0.0, -2.0]])
    ranges = kth_ranges(CIRCLE, xy, 1)
    assert ranges[7] == np.float32(np.hypot(-1.0, 0.001))  # just below +180
    assert ranges[0] == np.float32(np.hypot(-1.2, 0.001))  # just above -180
    assert ranges[2] == 2.0  # -90 deg
    assert np.count_nonzero(ranges < 4.0) == 3


def test_ranges_outside_limits_are_ignored():
    """Points closer than range_min or beyond range_max do not count."""
    xy = np.array([[0.05, 0.0], [9.0, 0.0]])
    assert (kth_ranges(CIRCLE, xy, 1) == 4.0).all()


def test_kth_nearest_ignores_a_single_spurious_return():
    """With k = 2 a lone dust return is ignored; the second nearest point of a wall counts."""
    xy = np.array([[0.5, 0.0], [2.0, 0.0], [2.1, 0.01], [2.2, -0.01]])
    assert kth_ranges(FRONT, xy, 1)[2] == pytest.approx(0.5)
    assert kth_ranges(FRONT, xy, 2)[2] == pytest.approx(np.hypot(2.0, 0.0), abs=1e-6)
    assert kth_ranges(FRONT, xy, 3)[2] == pytest.approx(np.hypot(2.1, 0.01), abs=1e-6)
    assert kth_ranges(FRONT, xy[:1], 2)[2] == 8.0  # fewer than k points: free


def test_kth_matches_a_brute_force_reference():
    """Random clouds: every sector equals the k-th smallest range computed per sector."""
    rng = default_rng(3)
    layout = SectorLayout.circle(36, 0.2, 5.0)
    for k in (1, 2, 4):
        xy = rng.uniform(-6, 6, (500, 2))
        ranges = kth_ranges(layout, xy, k)
        r = np.hypot(xy[:, 0], xy[:, 1])
        theta = np.mod(np.arctan2(xy[:, 1], xy[:, 0]) + math.pi, 2 * math.pi)
        for i in range(36):
            sel = np.sort(r[((theta // layout.increment) == i) & (r >= 0.2) & (r <= 5.0)])
            expected = sel[k - 1] if len(sel) >= k else 5.0
            assert ranges[i] == pytest.approx(expected, abs=1e-5)


def test_layouts_expose_the_sector_count_and_blocked():
    """The sector count is configurable; blocked() is all BLOCKED, free() all range_max."""
    layout = SectorLayout.circle(72, 0.1, 4.0)
    assert layout.full_circle and not FRONT.full_circle
    assert layout.increment == pytest.approx(math.radians(5.0))
    assert set(layout.blocked().tolist()) == {BLOCKED}
    assert set(layout.free().tolist()) == {4.0}
    assert len(kth_ranges(SectorLayout.front(math.pi, 7, 0.1, 8.0), np.empty((0, 2)))) == 7


def test_locate_returns_input_rows():
    """locate() reports which input rows were binned."""
    xy = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, 0.0], [1.0, 0.5]])
    rows, idx, r = FRONT.locate(xy)
    assert rows.tolist() == [1, 3]
    assert idx.tolist() == [2, 2]
    assert r == pytest.approx([1.0, np.hypot(1.0, 0.5)])


@pytest.mark.parametrize('fov_deg', [200.0, 300.0, 359.0])
def test_wide_front_fov_keeps_points_beyond_90_deg(fov_deg):
    """A front fov above 180 deg sees points beside and behind up to +-fov / 2."""
    layout = SectorLayout.front(math.radians(fov_deg), 36, 0.1, 8.0)
    inside = 0.5 * fov_deg - 5.0
    outside = 0.5 * fov_deg + 0.25 * (360.0 - fov_deg)  # in the gap behind the robot
    angles = np.radians([0.0, 95.0, inside, -95.0, -inside, outside])
    xy = np.column_stack((np.cos(angles), np.sin(angles)))
    rows, idx, _ = layout.locate(xy)
    assert rows.tolist() == [0, 1, 2, 3, 4]
    assert idx[2] == 35 and idx[4] == 0
