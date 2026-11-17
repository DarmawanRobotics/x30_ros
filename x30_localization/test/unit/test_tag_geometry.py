"""Review #15: tag geometry with the real optical frame rotation, hand-computed poses."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_localization.core.sources.tag import (
    TagNoise,
    TagSource,
    map_base_from_tag,
    map_tag_from_base,
    tag_covariance,
    view_angle,
)
from x30_utils.math import Pose2D, make_transform, planar_transform, quat_from_rpy

C30, S30 = math.cos(math.radians(30.0)), math.sin(math.radians(30.0))


def frame(x_axis, y_axis, z_axis, xyz):
    """Return a 4x4 transform from its axes (columns) and origin, written by hand."""
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x_axis, y_axis, z_axis, xyz
    return m


# URDF camera joint: 0.25 m ahead, 0.1 m up, optical rotation rpy (-pi/2, 0, -pi/2)
BASE_CAM = make_transform((0.25, 0.0, 0.1), quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2))


def test_optical_rotation_is_right_down_forward():
    """Camera x = base right (-y), y = base down (-z), z = base forward (x)."""
    expected = frame((0, -1, 0), (0, 0, -1), (1, 0, 0), (0.25, 0.0, 0.1))
    assert np.allclose(BASE_CAM, expected)


def test_tag_head_on_robot_yawed_30_degrees():
    """Robot at (1, -2, 30 deg); tag 1.5 m ahead on the optical axis, facing the camera."""
    cam_tag = frame((-1, 0, 0), (0, 1, 0), (0, 0, -1), (0.0, 0.0, 1.5))
    # by hand: the tag centre is 0.25 + 1.5 m along the heading, 0.1 m up; its z faces back,
    # its x points to the robot's left, its y down
    map_tag = frame(
        (-S30, C30, 0),
        (0, 0, -1),
        (-C30, -S30, 0),
        (1.0 + 1.75 * C30, -2.0 + 1.75 * S30, 0.1),
    )
    pose = Pose2D.from_matrix(map_base_from_tag(map_tag, BASE_CAM, cam_tag))
    assert pose.to_array() == pytest.approx([1.0, -2.0, math.radians(30.0)], abs=1e-9)
    assert view_angle(cam_tag) == pytest.approx(0.0, abs=1e-9)


def test_tag_off_axis_and_rotated_robot_yawed_90_degrees():
    """Robot at (2, 1, 90 deg); tag 2 m ahead, 0.3 m left, 0.2 m up, turned 20 deg."""
    a = math.radians(20.0)
    ca, sa = math.cos(a), math.sin(a)
    # in the camera: rotated by a about the camera y axis (vertical, pointing down)
    cam_tag = frame((ca, 0, -sa), (0, 1, 0), (sa, 0, ca), (-0.3, -0.2, 2.0))
    # by hand in the map: facing +y, the camera is at (2, 1.25, 0.1), right = +x, down = -z,
    # forward = +y; the tag is at (2 - 0.3, 1.25 + 2, 0.1 + 0.2)
    map_tag = frame((ca, -sa, 0), (0, 0, -1), (sa, ca, 0), (1.7, 3.25, 0.3))
    m_map_base = map_base_from_tag(map_tag, BASE_CAM, cam_tag)
    assert Pose2D.from_matrix(m_map_base).to_array() == pytest.approx(
        [2.0, 1.0, math.radians(90.0)], abs=1e-9
    )
    assert m_map_base[2, 3] == pytest.approx(0.0, abs=1e-9)
    # recording is the inverse composition: the same scene gives the tag back
    robot = planar_transform(2.0, 1.0, math.radians(90.0))
    assert np.allclose(map_tag_from_base(robot, BASE_CAM, cam_tag), map_tag)


def test_tilted_body_still_gives_the_planar_pose():
    """A pitched base (gait) changes T_cam_tag but the planar pose stays right."""
    robot = make_transform((3.0, 4.0, 0.0), quat_from_rpy(0.0, math.radians(5.0), -2.0))
    map_tag = frame((0, 1, 0), (0, 0, -1), (-1, 0, 0), (6.0, 3.0, 0.5))
    cam_tag = np.linalg.inv(robot @ BASE_CAM) @ map_tag
    pose = Pose2D.from_matrix(map_base_from_tag(map_tag, BASE_CAM, cam_tag))
    assert pose.to_array() == pytest.approx([3.0, 4.0, -2.0], abs=1e-9)


def test_measurement_noise_grows_with_distance_and_view_angle():
    source = TagSource(TagNoise(max_distance_m=10.0))
    head_on = frame((-1, 0, 0), (0, 1, 0), (0, 0, -1), (0.0, 0.0, 1.0))
    map_tag = planar_transform(5.0, 0.0, math.pi)
    near = source.measure('t', map_tag, BASE_CAM, head_on, 0.0)
    far_cam = head_on.copy()
    far_cam[2, 3] = 3.0
    far = source.measure('t', map_tag, BASE_CAM, far_cam, 0.0)
    assert far.covariance[0, 0] > 9.0 * near.covariance[0, 0] * 0.99
    assert far.source == 'tag:t'
    assert far.valid() and near.valid()


def test_heading_error_moves_the_robot_around_the_tag():
    """With the tag 2 m ahead (+x), a heading error moves the robot sideways (y)."""
    cov = tag_covariance((2.0, 0.0), 2.0, 0.0, TagNoise())
    assert cov[1, 1] > cov[0, 0]
    assert abs(cov[1, 2]) > 0.0 and cov[0, 2] == pytest.approx(0.0)
    assert np.all(np.linalg.eigvalsh(cov) > 0.0)


def test_far_and_oblique_detections_are_refused():
    source = TagSource(TagNoise(max_distance_m=3.0, max_view_angle=math.radians(45.0)))
    map_tag = np.eye(4)
    far = frame((-1, 0, 0), (0, 1, 0), (0, 0, -1), (0.0, 0.0, 3.5))
    assert 'too far' in source.measure('a', map_tag, BASE_CAM, far, 0.0)
    b = math.radians(60.0)
    oblique = frame((math.cos(b), 0, -math.sin(b)), (0, 1, 0), (math.sin(b), 0, math.cos(b)),
                    (0.0, 0.0, 1.0))  # fmt: skip
    assert 'obliquely' in source.measure('a', map_tag, BASE_CAM, oblique, 0.0)
