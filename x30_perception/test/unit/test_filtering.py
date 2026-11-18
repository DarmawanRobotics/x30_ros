"""Self filter, leveling and the height band relative to the floor (finding P1)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_perception.core.config import FilterConfig
from x30_perception.core.filtering import (
    ground_offsets,
    level,
    obstacle_points,
    self_mask,
    tilt_rotation,
)
from x30_utils.math import make_transform, quat_from_rpy
from x30_utils.robot import Footprint

FOOT = Footprint(front=0.4, rear=0.4, half_width=0.28)
BASE_HEIGHT = 0.35


def floor_points(base_height: float, pitch: float, roll: float = 0.0) -> np.ndarray:
    """A flat floor 0.5-6 m around the robot, as seen in a base_link pitched/rolled by the gait."""
    xs, ys = np.meshgrid(np.linspace(-6, 6, 121), np.linspace(-6, 6, 121))
    world = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, -base_height)))
    world = world[np.hypot(world[:, 0], world[:, 1]) > 0.5]
    tilt = make_transform(quat=quat_from_rpy(roll, pitch, 0.0))[:3, :3]
    return world @ tilt  # base_link = tilt^T * level


def tilt_of_body(pitch: float, roll: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """odom->base with the body pitched and rolled (and some heading)."""
    return make_transform((3.0, -1.0, 0.35), quat_from_rpy(roll, pitch, yaw))


def test_pitched_body_does_not_see_the_floor():
    """2 deg of pitch: base_link filtering would put the floor 1.4-2.9 m ahead in the band."""
    pitch = math.radians(2.0)
    points = floor_points(BASE_HEIGHT, pitch)
    config = FilterConfig()
    naive = obstacle_points(points, FOOT, BASE_HEIGHT, config, None)
    assert len(naive.xy) > 100  # the defect: floor returns become obstacles
    for yaw in (0.0, 1.0, -2.5):
        tilt = tilt_rotation(tilt_of_body(pitch, yaw=yaw))
        assert len(obstacle_points(points, FOOT, BASE_HEIGHT, config, tilt).xy) == 0


def test_rolled_and_pitched_body_still_sees_a_box():
    """A 20 cm box 2 m ahead stays an obstacle while the body rolls and pitches."""
    pitch, roll = math.radians(4.0), math.radians(-3.0)
    tilt_m = make_transform(quat=quat_from_rpy(roll, pitch, 0.0))[:3, :3]
    box = np.array([[2.0, y, z - BASE_HEIGHT] for y in (-0.1, 0.0, 0.1) for z in (0.12, 0.2)])
    points = np.vstack((floor_points(BASE_HEIGHT, pitch, roll), box @ tilt_m))
    tilt = tilt_rotation(tilt_of_body(pitch, roll, yaw=0.7))
    obstacles = obstacle_points(points, FOOT, BASE_HEIGHT, FilterConfig(), tilt)
    assert len(obstacles.xy) == 6
    assert obstacles.xy[:, 0] == pytest.approx(2.0, abs=1e-6)
    assert obstacles.height.min() == pytest.approx(0.12, abs=1e-6)


def test_band_is_relative_to_the_floor_through_base_height():
    """The same point is floor for a tall robot and an obstacle for a short one."""
    point = np.array([[2.0, 0.0, -0.3]])
    config = FilterConfig(height_min=0.1)
    assert len(obstacle_points(point, FOOT, 0.30, config, None).xy) == 0  # 0.0 m above floor
    assert len(obstacle_points(point, FOOT, 0.45, config, None).xy) == 1  # 0.15 m above
    high = np.array([[2.0, 0.0, 1.5]])
    assert len(obstacle_points(high, FOOT, 0.35, config, None).xy) == 0  # above height_max


def test_tilt_rotation_removes_yaw_only():
    """tilt_rotation keeps roll and pitch and drops the heading."""
    m = tilt_of_body(0.1, -0.05, yaw=2.0)
    expected = make_transform(quat=quat_from_rpy(-0.05, 0.1, 0.0))[:3, :3]
    np.testing.assert_allclose(tilt_rotation(m), expected, atol=1e-12)
    np.testing.assert_allclose(level(np.ones((2, 3)), None), np.ones((2, 3)))


def test_self_filter_uses_the_footprint_and_margin():
    """Points on the robot body (footprint + margin) are dropped, others kept."""
    points = np.array([[0.1, 0.1, 0.0], [0.43, 0.0, 0.0], [0.5, 0.0, 0.0], [-0.42, 0.3, 0.0]])
    assert self_mask(points, FOOT, 0.05).tolist() == [True, True, False, True]
    assert self_mask(points, FOOT, 0.0).tolist() == [True, False, False, False]


def test_ground_segmentation_follows_a_ramp_but_not_an_obstacle():
    """A 4 cm ramp step is floor; a box on it stays; a box without floor around stays too."""
    ramp = np.array([[x, y, 0.04] for x in np.arange(2.0, 3.0, 0.1) for y in (-0.2, 0.0, 0.2)])
    xy, height = ramp[:, :2], ramp[:, 2]
    np.testing.assert_allclose(ground_offsets(xy, height, 0.3, 0.05), 0.04)
    lone = np.array([[4.0, 0.0], [4.05, 0.0]])
    np.testing.assert_allclose(ground_offsets(lone, np.array([0.2, 0.3]), 0.3, 0.05), 0.0)
    config = FilterConfig(ground_segmentation=True, height_min=0.08)
    pts = np.vstack((ramp, [[2.05, 0.05, 0.2]])) - [0.0, 0.0, BASE_HEIGHT]
    obstacles = obstacle_points(pts, FOOT, BASE_HEIGHT, config, None)
    assert len(obstacles.xy) == 1 and obstacles.height[0] == pytest.approx(0.16)
    assert len(ground_offsets(np.empty((0, 2)), np.empty(0), 0.3, 0.05)) == 0


def test_config_rejects_an_inverted_band():
    """height_min must be below height_max."""
    assert FilterConfig(height_min=0.5, height_max=0.2).validate()


def test_nested_validation_runs_through_build():
    """build() of the whole configuration reports problems of nested sections."""
    from x30_perception.core.config import PerceptionConfig
    from x30_utils.config import ConfigError, build

    with pytest.raises(ConfigError, match=r'filter\.height_min'):
        build(PerceptionConfig, {'filter.height_min': 2.0, 'filter.height_max': 1.0})
    with pytest.raises(ConfigError, match=r'safety\.range_min'):
        build(PerceptionConfig, {'safety.range_min': 5.0})
    assert build(PerceptionConfig).safety.num_sectors == 72
