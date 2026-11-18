"""Log-odds grid: hits occupy, rays carve free space, nothing behind empty bins, capped growth."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_mapping.core import map_io, pgm
from x30_mapping.core.config import GridConfig
from x30_mapping.core.grid import LogOddsGrid
from x30_mapping.core.raycast import ray_cells


def make(resolution=0.05, size=10.0, max_size=150.0, grow=5.0) -> LogOddsGrid:
    return LogOddsGrid(
        GridConfig(resolution=resolution, initial_size_m=size, max_size_m=max_size, grow_m=grow)
    )


def wall_scan(x=3.0, half=2.0, spacing=0.02):
    ys = np.arange(-half, half + 1e-9, spacing)
    return np.column_stack([np.full_like(ys, x), ys])


def cell(grid, x, y):
    col, row = grid.spec.cells(np.array([[x, y]]))[0]
    return int(grid.occupancy()[row, col])


def test_wall_is_occupied_and_space_before_it_free():
    """After a few scans the wall is occupied, the corridor before it free, behind unknown."""
    grid = make()
    for _ in range(3):
        grid.integrate((0.0, 0.0), wall_scan())
    assert cell(grid, 3.0, 0.0) == pgm.OCCUPIED
    assert cell(grid, 1.5, 0.0) == pgm.FREE
    assert cell(grid, 2.0, 1.0) == pgm.FREE
    assert cell(grid, 4.0, 0.0) == pgm.UNKNOWN
    assert cell(grid, -2.0, 0.0) == pgm.UNKNOWN  # no hit behind the robot: nothing carved
    assert grid.scans == 3
    assert grid.known_area() > 3.0


def test_a_person_walking_through_is_erased_again():
    """Free space carved through a cell that was hit once turns it back into free."""
    grid = make()
    grid.integrate((0.0, 0.0), np.array([[1.5, 0.0]]))
    assert cell(grid, 1.5, 0.0) == pgm.OCCUPIED
    for _ in range(4):
        grid.integrate((0.0, 0.0), wall_scan())
    assert cell(grid, 1.5, 0.0) == pgm.FREE


def test_range_limits_and_non_finite_points_are_ignored():
    """Points beyond max_range, inside min_range or not finite change nothing."""
    grid = make(resolution=0.1, size=4.0)
    points = np.array([[30.0, 0.0], [0.05, 0.0], [np.nan, 1.0]])
    used = grid.integrate((0.0, 0.0), points, max_range=15.0, min_range=0.2)
    assert used == 0 and grid.scans == 0
    assert (grid.occupancy() == pgm.UNKNOWN).all()


def test_grid_grows_without_moving_cells():
    """Hits outside the grid grow it in whole cells; old cells keep their map position."""
    grid = make(resolution=0.1, size=2.0)
    grid.integrate((0.0, 0.0), np.array([[0.5, 0.5]]))
    width = grid.width
    grid.integrate((0.0, 0.0), np.array([[-7.0, 4.0]]), max_range=20.0)
    assert grid.width > width
    assert cell(grid, 0.55, 0.55) == pgm.OCCUPIED
    assert cell(grid, -6.95, 4.05) == pgm.OCCUPIED
    ox, oy = grid.origin_x / 0.1, grid.origin_y / 0.1
    assert ox == pytest.approx(round(ox)) and oy == pytest.approx(round(oy))


def test_growth_is_capped_on_a_pose_jump():
    """#8: a 300 m jump grows the grid to max_size_m at most; far hits are dropped."""
    grid = make(resolution=0.05, size=20.0, max_size=50.0)
    grid.integrate((0.0, 0.0), wall_scan())
    grid.integrate((300.0, 0.0), wall_scan(x=303.0), max_range=15.0)
    assert grid.width <= 1000 and grid.height <= 1000
    assert grid.logodds.nbytes <= 1000 * 1000 * 4
    assert grid.capped
    assert cell(grid, 3.0, 0.0) == pgm.OCCUPIED  # the old map is still there
    grid.integrate((0.0, 0.0), np.array([[-12.0, 0.0]]))  # no room left on any side
    assert grid.width <= 1000 and grid.origin_x == pytest.approx(-10.0)


def test_cap_splits_the_room_when_both_sides_grow():
    grid = make(resolution=1.0, size=4.0, max_size=10.0, grow=1.0)
    grid.ensure(-50.0, -1.0, 50.0, 1.0)
    assert grid.width == 10 and grid.capped


def test_prior_round_trip_and_rejections():
    """A saved map loads back as the prior; another resolution or a rotated origin is refused."""
    grid = make()
    for _ in range(3):
        grid.integrate((0.0, 0.0), wall_scan())
    saved = map_io.crop_known(grid.to_grid_map(), 0.5)
    other = make(size=4.0)
    other.load_prior(saved)
    assert cell(other, 3.0, 0.0) == pgm.OCCUPIED
    assert cell(other, 1.0, 0.0) == pgm.FREE
    with pytest.raises(ValueError):
        make(resolution=0.1).load_prior(saved)
    with pytest.raises(ValueError):
        other.load_prior(map_io.GridMap(saved.data, 0.05, 0.0, 0.0, math.pi / 2))


def test_ray_cells_stop_at_the_reach():
    """ray_cells covers the cells from the sensor up to the reach, not past it."""
    grid = make(resolution=0.1, size=4.0)
    cells = ray_cells(grid.spec, 0.05, 0.05, np.array([0.0]), np.array([0.95]))
    cols = sorted(int(c) % grid.width - round(-grid.origin_x / 0.1) for c in cells)
    assert cols == list(range(10))
