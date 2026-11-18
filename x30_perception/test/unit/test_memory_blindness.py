"""Obstacle memory in odom (finding B4) and blind detection."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_perception.core.blindness import check_blind
from x30_perception.core.memory import NearField, ObstacleMemory
from x30_utils.math import Pose2D, make_transform, quat_from_rpy

MOUNT = make_transform((0.13, 0.0, 0.18))  # sensor 0.18 m above base_link, level
VIEW = NearField.of_mount(MOUNT, 0.35, math.radians(-7.0))


def test_near_field_zone_of_a_level_mid360():
    """Sensor 0.53 m above the floor, -7 deg: a 10 cm object is hidden within ~3.5 m."""
    assert VIEW.sensor_height == pytest.approx(0.53)
    assert VIEW.radius(0.10) == pytest.approx(0.43 / math.tan(math.radians(7.0)))
    assert VIEW.visible_height(np.array([[0.13, 0.0]]))[0] == pytest.approx(0.53)
    pitched = NearField.of_mount(
        make_transform((0.13, 0.0, 0.18), quat_from_rpy(0.0, math.radians(10.0), 0.0)),
        0.35,
        math.radians(-7.0),
    )
    # pitched down in front, up behind: the worst azimuth (behind) sees less steeply
    assert pitched.descent < VIEW.descent
    up = NearField.of_mount(
        make_transform(quat=quat_from_rpy(0.0, math.radians(-10.0), 0.0)), 0.35, -0.05
    )
    assert math.isinf(up.radius(0.1))


def test_memory_keeps_an_object_the_sensor_no_longer_sees():
    """A box seen 2 m ahead stays at the right place while the robot walks up to it."""
    memory = ObstacleMemory(memory_sec=2.0, radius=5.0, resolution=0.05, max_cells=1000)
    box = np.array([[2.0, 0.0], [2.0, 0.04]])
    memory.update(Pose2D(1.0, 1.0, math.pi / 2), box, np.array([0.12, 0.15]), now=0.0)
    # robot walked 1.2 m forward (odom +y) and turned left 90 deg; the box left the view
    pose = Pose2D(1.0, 2.2, math.pi)
    memory.update(pose, np.empty((0, 2)), np.empty(0), now=1.0)
    xy = memory.hidden(pose, now=1.0, view=VIEW)
    # box in odom at (1.0, 3.0..3.04); seen from (1.0, 2.2) facing -x it is ~0.8 m to the right
    assert len(xy) == 2
    np.testing.assert_allclose(xy[:, 1], -0.8, atol=0.06)
    np.testing.assert_allclose(xy[:, 0], 0.0, atol=0.06)


def test_memory_decays_after_memory_sec():
    """Remembered points disappear memory_sec after they were last seen."""
    memory = ObstacleMemory(memory_sec=1.0, radius=5.0, resolution=0.05, max_cells=1000)
    memory.update(Pose2D(), np.array([[1.0, 0.0]]), np.array([0.1]), now=10.0)
    assert len(memory.hidden(Pose2D(), now=10.9, view=VIEW)) == 1
    assert len(memory.hidden(Pose2D(), now=11.1, view=VIEW)) == 0
    assert len(memory) == 0
    memory.update(Pose2D(), np.array([[1.0, 0.0]]), np.array([0.1]), now=20.0)
    memory.update(Pose2D(), np.array([[1.0, 0.0]]), np.array([0.1]), now=20.8)  # seen again
    assert len(memory.hidden(Pose2D(), now=21.5, view=VIEW)) == 1


def test_visible_tall_objects_are_not_merged():
    """A person 2 m away is in the live view: the memory does not keep a trail of them."""
    memory = ObstacleMemory(memory_sec=5.0, radius=5.0, resolution=0.05, max_cells=1000)
    memory.update(Pose2D(), np.array([[2.0, 0.0], [0.5, 0.0]]), np.array([1.6, 0.1]), now=0.0)
    hidden = memory.hidden(Pose2D(), now=0.5, view=VIEW)
    np.testing.assert_allclose(hidden, [[0.525, 0.025]], atol=1e-9)  # only the low one


def test_memory_radius_and_size_bound():
    """Points beyond radius are not remembered; the oldest cells go beyond max_cells."""
    memory = ObstacleMemory(memory_sec=5.0, radius=1.0, resolution=0.1, max_cells=3)
    memory.update(Pose2D(), np.array([[3.0, 0.0]]), np.array([0.1]), now=0.0)
    assert len(memory) == 0
    for t, x in enumerate((0.25, 0.45, 0.65, 0.85)):
        memory.update(Pose2D(), np.array([[x, 0.0]]), np.array([0.1]), now=float(t))
    assert len(memory) == 3
    xy = memory.hidden(Pose2D(), now=3.0, view=VIEW)
    assert sorted(np.round(xy[:, 0], 2).tolist()) == [0.45, 0.65, 0.85]
    memory.clear()
    assert len(memory) == 0


def test_blind_when_too_few_points():
    """Fewer than min_points valid points: blind."""
    assert check_blind(np.ones((10, 3)), 100).blind
    assert not check_blind(np.ones((100, 3)), 100).blind
    assert check_blind(np.empty((0, 3)), 1).blind


def test_blind_when_a_region_is_covered():
    """A covered quarter of the view is blind even with plenty of points elsewhere."""
    angles = np.linspace(-math.pi, math.pi, 400, endpoint=False)
    ring = np.column_stack((np.cos(angles), np.sin(angles), np.zeros_like(angles)))
    assert not check_blind(ring, 100, regions=4, min_region_points=50).blind
    covered = ring[~((angles >= 0.0) & (angles < math.pi / 2))]
    result = check_blind(covered, 100, regions=4, min_region_points=50)
    assert result.blind and 'region 2' in result.reason
    assert not check_blind(covered, 100, regions=4, min_region_points=0).blind
