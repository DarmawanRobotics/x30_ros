"""Scan integration: band, self filter, jump and correction gates, refinement (M1, #8, #14)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_mapping.core import pgm
from x30_mapping.core.config import GridConfig, MapperConfig, RefineConfig
from x30_mapping.core.grid import LogOddsGrid
from x30_mapping.core.integrator import Outcome, Scan, ScanIntegrator, correction_in_flight
from x30_utils.math import inverse, make_transform, planar_transform, transform_points
from x30_utils.robot import RobotProfile

BASE_HEIGHT = 0.35
MOUNT = make_transform((0.1, 0.0, 0.2))  # base <- sensor


def room_points(width=8.0, depth=6.0, spacing=0.03, heights=(0.3, 1.0)) -> np.ndarray:
    """World points (map frame, z above the floor) of a rectangular room with a pillar."""
    xs = np.arange(-width / 2, width / 2, spacing)
    ys = np.arange(-depth / 2, depth / 2, spacing)
    walls = [np.column_stack((xs, np.full_like(xs, y))) for y in (-depth / 2, depth / 2)]
    walls += [np.column_stack((np.full_like(ys, x), ys)) for x in (-width / 2, width / 2)]
    a = np.linspace(0, 2 * math.pi, 60, endpoint=False)
    walls.append(np.column_stack((1.5 + 0.2 * np.cos(a), 1.0 + 0.2 * np.sin(a))))
    xy = np.vstack(walls)
    return np.vstack([np.column_stack((xy, np.full(len(xy), h))) for h in heights])


def corridor_points(length=30.0, half=1.0, spacing=0.03) -> np.ndarray:
    xs = np.arange(-length / 2, length / 2, spacing)
    xy = np.vstack([np.column_stack((xs, np.full_like(xs, y))) for y in (-half, half)])
    return np.column_stack((xy, np.full(len(xy), 0.5)))


def make_scan(world, x, y, yaw, stamp, map_odom=None, window=()) -> Scan:
    """The world as seen by the sensor of a robot at (x, y, yaw), odom = map unless given."""
    map_odom = np.eye(4) if map_odom is None else map_odom
    map_base = planar_transform(x, y, yaw, BASE_HEIGHT)
    map_sensor = map_base @ MOUNT
    points = transform_points(inverse(map_sensor), world)
    odom_sensor = inverse(map_odom) @ map_sensor
    return Scan(points, stamp, MOUNT, map_odom, odom_sensor, tuple(window))


def integrator(refine=False, max_size=60.0) -> ScanIntegrator:
    config = MapperConfig(
        grid=GridConfig(initial_size_m=10.0, max_size_m=max_size),
        refine=RefineConfig(enabled=refine),
    )
    robot = RobotProfile(base_height=BASE_HEIGHT)
    return ScanIntegrator(config, robot, LogOddsGrid(config.grid))


def value(grid, x, y) -> int:
    col, row = grid.spec.cells(np.array([[x, y]]))[0]
    return int(grid.occupancy()[row, col])


def test_band_drops_floor_ceiling_and_the_robot():
    """O1: the band is above the floor in the map frame; legs inside the footprint are dropped."""
    world = room_points()
    floor = np.column_stack((np.arange(0.5, 3.0, 0.05), np.zeros(50), np.zeros(50)))
    ceiling = np.column_stack((np.arange(0.5, 3.0, 0.05), np.zeros(50), np.full(50, 2.5)))
    legs = np.array([[0.25, 0.2, 0.1], [-0.25, -0.2, 0.1]])
    integ = integrator()
    for k in range(3):
        result = integ.process(make_scan(np.vstack((world, floor, ceiling, legs)), 0, 0, 0, k))
        assert result.outcome is Outcome.INTEGRATED
    grid = integ.grid
    assert value(grid, 4.0, 0.0) == pgm.OCCUPIED
    assert value(grid, 1.52, 0.02) == pgm.FREE  # floor below the band
    assert value(grid, 0.27, 0.22) != pgm.OCCUPIED  # own legs


def test_jump_is_rejected_and_never_grows_the_grid():
    """#8: a 300 m jump is rejected, the grid keeps its size; back on the map it resumes."""
    world = room_points()
    integ = integrator()
    for k in range(3):
        assert (
            integ.process(make_scan(world, 0.0, 0.0, 0.0, 0.5 * k)).outcome is Outcome.INTEGRATED
        )
    shape = integ.grid.logodds.shape
    far = world + np.array([300.0, 0.0, 0.0])
    jumped = integ.process(make_scan(far, 300.0, 0.0, 0.0, 1.5))
    assert jumped.outcome is Outcome.JUMP and 'jumped' in jumped.message
    for k in range(5):  # consistent with each other, but not with the map: still held
        result = integ.process(make_scan(far, 300.0 + 0.1 * k, 0.0, 0.0, 2.0 + 0.5 * k))
        assert result.outcome is Outcome.HELD
    assert integ.grid.logodds.shape == shape
    assert integ.rejected == 6
    back = integ.process(make_scan(world, 0.1, 0.0, 0.0, 5.0))  # relocalized on the map
    assert back.outcome is Outcome.INTEGRATED


def test_normal_motion_is_not_a_jump():
    """Motion within max_jump_m + max speed * dt passes, also after a gap in the scans."""
    world = room_points()
    integ = integrator()
    assert integ.process(make_scan(world, 0.0, 0.0, 0.0, 0.0)).outcome is Outcome.INTEGRATED
    assert integ.process(make_scan(world, 0.6, 0.0, 0.2, 0.5)).outcome is Outcome.INTEGRATED
    assert integ.process(make_scan(world, 2.5, 0.5, 0.2, 4.0)).outcome is Outcome.INTEGRATED


def test_scans_during_a_correction_are_skipped():
    """#14: map->odom changing around the scan stamp skips it; a steady one integrates."""
    world = room_points()
    integ = integrator()
    old, new = np.eye(4), planar_transform(0.2, 0.0, 0.0)
    skipped = integ.process(make_scan(world, 0.0, 0.0, 0.0, 0.0, map_odom=new, window=(old, new)))
    assert skipped.outcome is Outcome.CORRECTION and skipped.rejected
    assert integ.grid.scans == 0
    steady = integ.process(make_scan(world, 0.0, 0.0, 0.0, 0.5, map_odom=new, window=(new, new)))
    assert steady.outcome is Outcome.INTEGRATED
    assert correction_in_flight((old, planar_transform(0.0, 0.0, 0.02)), 0.02, math.radians(0.5))
    assert not correction_in_flight((old, planar_transform(0.01, 0.0, 0.0)), 0.02, 0.01)


def wall_sharpness(grid, world_xy) -> float:
    """Return the fraction of occupied cells within one cell of a true wall point."""
    occupied = np.argwhere(grid.occupied_mask())
    centers = grid.spec.centers(occupied[:, ::-1])
    truth = set(map(tuple, grid.spec.cells(world_xy)))
    cells = grid.spec.cells(centers)
    assert len(cells) == len(centers)
    near = 0
    for c, r in cells:
        if any((c + dc, r + dr) in truth for dc in (-1, 0, 1) for dr in (-1, 0, 1)):
            near += 1
    return near / max(len(occupied), 1)


@pytest.mark.parametrize('refine', [False, True])
def test_refinement_keeps_walls_sharp_under_pose_error(refine):
    """M1: TF poses off by 8 cm / 1.5 deg smear the walls; the refinement keeps them sharp."""
    world = room_points()
    integ = integrator(refine=refine)
    for k in range(4):
        integ.process(make_scan(world, 0.05 * k, 0.0, 0.0, 0.5 * k))  # build a good map
    errors = [(0.08, -0.05, 1.5), (-0.07, 0.06, -1.5), (0.06, 0.08, 1.0), (-0.08, -0.06, -1.2)]
    for k, (ex, ey, eyaw) in enumerate(errors):
        true_pose = (0.3 + 0.1 * k, 0.1, 0.05)
        scan = make_scan(world, *true_pose, 3.0 + 0.5 * k)
        tf = planar_transform(
            true_pose[0] + ex, true_pose[1] + ey, true_pose[2] + math.radians(eyaw)
        )
        tf_sensor = tf @ planar_transform(0, 0, 0, BASE_HEIGHT) @ MOUNT
        result = integ.process(Scan(scan.points, scan.stamp, MOUNT, np.eye(4), tf_sensor))
        assert result.outcome is Outcome.INTEGRATED
        assert (result.correction is not None) == refine
    sharpness = wall_sharpness(integ.grid, room_points(heights=(0.3,))[:, :2])
    if refine:
        assert sharpness > 0.97
    else:
        assert sharpness < 0.97


def test_refinement_refuses_an_ambiguous_corridor():
    """M1 gate: along a featureless corridor the match is ambiguous and the TF pose is kept."""
    world = corridor_points()
    integ = integrator(refine=True)
    for k in range(4):
        integ.process(make_scan(world, 0.2 * k, 0.0, 0.0, 0.5 * k))
    result = integ.process(make_scan(world, 1.0, 0.0, 0.0, 2.5))
    assert result.outcome is Outcome.INTEGRATED
    assert result.correction is None and 'ambiguous' in result.message
