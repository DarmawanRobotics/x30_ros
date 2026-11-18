"""LiDAR model: tilted Mid-360 rings, ranges, floor, dropout, the X30's four mounts, density."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from x30_sim.core.config import X30_LIDARS, LidarConfig
from x30_sim.core.lidar import LidarModel, dome_rotation, sensor_rays
from x30_sim.core.world import World
from x30_utils.math import Pose2D, default_rng

WORLDS = Path(__file__).resolve().parents[2] / 'config' / 'worlds'
BASE_HEIGHT = 0.35
LEVEL = (0.0, 0.0, 0.15, 0.0, 90.0)  # one Mid-360 lying flat 0.5 m above the floor
EXACT = LidarConfig(mounts=LEVEL, noise_m=0.0, floor_returns=False)
X30 = LidarConfig(noise_m=0.0)
STANDING = 0.48  # the X30's base_height


def model(config: LidarConfig = EXACT, seed: int = 0, base: float = BASE_HEIGHT) -> LidarModel:
    """A model of config (by default one level LiDAR 0.5 m above the floor)."""
    return LidarModel(config, base, default_rng(seed))


def ahead(points: np.ndarray, width: float = 0.05) -> np.ndarray:
    """Points straight ahead (|y| < width, x > 0)."""
    return points[(np.abs(points[:, 1]) < width) & (points[:, 0] > 0.0)]


def low_box(x: float, height: float = 0.1) -> np.ndarray:
    """A low circle on the floor x m ahead."""
    return np.array([[x, 0.0, 0.2, height]])


def test_dome_rotation_points_z_along_the_dome():
    for azimuth, elevation in ((0.0, 25.0), (0.0, -46.0), (180.0, -5.0), (90.0, 10.0)):
        a, e = math.radians(azimuth), math.radians(elevation)
        rotation = dome_rotation(azimuth, elevation)
        expected = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
        assert rotation @ [0.0, 0.0, 1.0] == pytest.approx(expected, abs=1e-12)
        assert rotation @ rotation.T == pytest.approx(np.eye(3), abs=1e-12)


def test_rings_span_the_vertical_field_of_view():
    rays = sensor_rays(EXACT)
    elevations = np.degrees(np.arcsin(rays[:, 2]))
    assert len(rays) == EXACT.rings * EXACT.azimuth_rays
    assert elevations.min() == pytest.approx(-7.0) and elevations.max() == pytest.approx(52.0)


def test_low_obstacle_hidden_below_the_lowest_beam():
    """The -7 deg beam from 0.5 m passes over a 10 cm box at 1 m, and hits it at 4 m."""
    lidar = model()
    assert len(ahead(lidar.scan(World(), Pose2D(), low_box(1.0)))) == 0
    far = ahead(lidar.scan(World(), Pose2D(), low_box(4.0)))
    assert len(far) == 1  # only the lowest ring reaches below 0.1 m there
    assert far[0, 0] == pytest.approx(3.8, abs=0.01)
    assert far[0, 2] == pytest.approx(0.15 - 3.8 * math.tan(math.radians(7.0)), abs=0.01)


def test_tall_wall_returns_every_ring_up_to_its_top():
    """A 2.5 m wall 1 m ahead returns each ring below its top (52 deg: 0.5 + 1.28 m)."""
    world = World.from_dict({'walls': [[[1.0, -5.0], [1.0, 5.0]]]})
    hits = ahead(model().scan(world, Pose2D()), width=0.01)
    assert len(hits) == EXACT.rings
    assert np.allclose(hits[:, 0], 1.0, atol=1e-3)
    short = World.from_dict({'walls': [{'points': [[1.0, -5.0], [1.0, 5.0]], 'height': 1.0}]})
    seen = ahead(model().scan(short, Pose2D()), width=0.01)
    assert 0 < len(seen) < EXACT.rings and seen[:, 2].max() <= 0.15 + 0.5 + 1e-6  # top at 1 m


def test_min_and_max_range():
    """Returns closer than range_min or beyond range_max are lost."""
    world = World.from_dict({'walls': [[[0.3, -5.0], [0.3, 5.0]]]})
    assert len(ahead(model(replace(EXACT, range_min=0.5)).scan(world, Pose2D()))) == 0
    assert len(ahead(model(replace(EXACT, range_min=0.1)).scan(world, Pose2D()))) > 0
    far = World.from_dict({'walls': [[[30.0, -5.0], [30.0, 5.0]]]})
    assert len(ahead(model(replace(EXACT, range_max=20.0)).scan(far, Pose2D()))) == 0


def test_floor_returns_of_the_downward_rings():
    """With floor returns the rings below the horizon hit the floor at h / tan(-e)."""
    points = model(replace(EXACT, floor_returns=True)).scan(World(), Pose2D())
    assert len(points) > 0 and np.allclose(points[:, 2], -BASE_HEIGHT, atol=1e-5)
    nearest = np.hypot(points[:, 0], points[:, 1]).min()  # the lowest ring, -7 deg
    assert nearest == pytest.approx(0.5 / math.tan(math.radians(7.0)), rel=1e-4)


def test_noise_and_dropout_are_seeded():
    """Noise and dropout follow the seed: same seed, same scan."""
    world = World.from_yaml(str(WORLDS / 'mall_loop.yaml'))
    noisy = LidarConfig(noise_m=0.02, point_dropout=0.3)
    a = model(noisy, seed=3).scan(world, world.start)
    b = model(noisy, seed=3).scan(world, world.start)
    full = model(X30, seed=3).scan(world, world.start)
    assert np.array_equal(a, b)
    assert len(a) == pytest.approx(0.7 * len(full), rel=0.05)


def test_points_are_in_base_link_whatever_the_mount():
    """A wall 2 m ahead is at x = 2 in base_link from a LiDAR mounted 0.5 m forward, turned."""
    world = World.from_dict({'walls': [[[2.0, -5.0], [2.0, 5.0]]]})
    shifted = replace(EXACT, mounts=(0.5, 0.2, 0.15, 30.0, 90.0))
    hits = ahead(model(shifted).scan(world, Pose2D()), width=0.3)
    assert len(hits) > 0 and np.allclose(hits[:, 0], 2.0, atol=1e-3)
    turned = model(shifted).scan(world, Pose2D(1.0, -1.0, math.pi / 2))  # the wall to the right
    right = turned[np.abs(turned[:, 0]) < 0.3]
    assert len(right) > 0 and np.allclose(right[:, 1], -1.0, atol=1e-3)


def test_x30_lidars_from_the_application_manual():
    """Origins in mm and the dome directions of 6.2: up / down at the front and at the rear."""
    millimetres = [
        (443.67, 0, 54.15),
        (432.97, 0, -55.19),
        (-465.76, 0, 34.84),
        (-410.74, 0, -56.01),
    ]
    mounts = np.array(X30.mount_list)
    assert np.allclose(mounts[:, :3], np.array(millimetres) / 1000.0)
    assert mounts[:, 3:].tolist() == [[0, 25], [0, -46], [180, -5], [180, -56]]
    assert X30.mount_list == list(X30_LIDARS)


def test_x30_sees_all_around_in_the_perception_band():
    """Pillars around the standing X30, 2 m away: each is seen 0.1 to 1.6 m above the floor."""
    angles = np.radians(np.arange(0.0, 360.0, 30.0))
    pillars = [[2.0 * math.cos(a), 2.0 * math.sin(a), 0.1] for a in angles]
    points = model(X30, base=STANDING).scan(World.from_dict({'pillars': pillars}), Pose2D())
    above_floor = points[:, 2] + STANDING
    band = points[(above_floor > 0.1) & (above_floor < 1.6)]
    bearing = np.arctan2(band[:, 1], band[:, 0])
    for a in angles:
        near = np.abs(np.remainder(bearing - a + math.pi, 2.0 * math.pi) - math.pi) < 0.1
        assert near.sum() >= 3, f'pillar at {math.degrees(a):.0f} deg unseen'


def test_floor_just_ahead_is_seen_from_the_front_edge_on():
    """The upper front LiDAR's lower rings cover the floor from 0.5 to 2.25 m ahead."""
    points = model(X30, base=STANDING).scan(World(), Pose2D())
    floor = points[np.abs(points[:, 2] + STANDING) < 1e-6]
    lane = floor[(np.abs(floor[:, 1]) < 0.25) & (floor[:, 0] > 0.0)]
    counts, _ = np.histogram(lane[:, 0], bins=np.arange(0.5, 2.26, 0.25))
    assert counts.min() >= 10, counts


def test_mall_world_is_dense_enough_for_perception():
    """Contract: >= 1000 points in perception's band (0.1-1.6 m above the floor) per scan."""
    world = World.from_yaml(str(WORLDS / 'mall_loop.yaml'))
    lidar = LidarModel(LidarConfig(), STANDING, default_rng(0))
    points = lidar.scan(world, world.start)
    above_floor = points[:, 2] + STANDING
    band = (above_floor > 0.1) & (above_floor < 1.6) & (np.hypot(points[:, 0], points[:, 1]) > 0.7)
    assert band.sum() >= 1000
