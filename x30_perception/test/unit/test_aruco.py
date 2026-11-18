"""ArUco detection and pose on a synthetic camera image (needs OpenCV with aruco)."""

from __future__ import annotations

import numpy as np
import pytest

from x30_perception.core.aruco import ArucoConfig, ArucoDetector, to_gray

cv2 = pytest.importorskip('cv2')
if not hasattr(cv2, 'aruco'):
    pytest.skip('OpenCV without the aruco module', allow_module_level=True)

from aruco_scene import K, render, yaw_rotation  # noqa: E402


def test_pose_of_a_configured_marker_in_the_stack_convention():
    t = np.array([0.10, -0.05, 1.2])
    image = render(4, yaw_rotation(np.radians(20.0)), t)
    poses = ArucoDetector(ArucoConfig()).detect(image, K, np.zeros(5))
    assert len(poses) == 1 and poses[0].frame == 'charging_dock'
    m = poses[0].matrix
    assert m[:3, 3] == pytest.approx(t, abs=0.01)
    z = m[:3, 2]  # normal toward the viewer (the camera looks along +z)
    # a 15 cm marker at 1.2 m: its normal is good to a few degrees (docking smooths it)
    assert z[2] < -0.9 and z[0] == pytest.approx(-np.sin(np.radians(20.0)), abs=0.08)
    assert m[0, 0] < -0.9 and m[1, 1] > 0.9  # x to the viewer's left, y down


def test_unconfigured_ids_and_far_markers_are_dropped():
    image = render(7, yaw_rotation(0.0), np.array([0.0, 0.0, 1.0]))
    assert ArucoDetector(ArucoConfig()).detect(image, K, None) == []
    near = render(4, yaw_rotation(0.0), np.array([0.0, 0.0, 1.0]))
    close_only = ArucoConfig(max_distance=0.5)
    assert ArucoDetector(close_only).detect(near, K, None) == []


def test_to_gray_reads_ros_encodings_with_row_padding():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[..., 1] = 200  # green
    padded = np.zeros((2, 12), dtype=np.uint8)
    padded[:, :9] = rgb.reshape(2, 9)
    gray = to_gray(padded.tobytes(), 3, 2, 12, 'rgb8')
    assert gray.shape == (2, 3) and int(gray[0, 0]) == round(0.587 * 200)
    assert to_gray(np.full(6, 9, np.uint8).tobytes(), 3, 2, 3, 'mono8').max() == 9
    with pytest.raises(ValueError, match='unsupported'):
        to_gray(b'', 1, 1, 2, 'mono16')


def test_config_needs_one_frame_and_size_per_id():
    assert ArucoConfig(ids=(1, 2), frames=('a',), sizes=(0.1,)).validate()
    assert not ArucoConfig().validate()
