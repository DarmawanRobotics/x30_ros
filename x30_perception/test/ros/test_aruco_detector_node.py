"""aruco_detector_node over ROS 1 topics: camera_info (K, D) and a mono8 image -> a TF frame."""

from __future__ import annotations

import numpy as np
import pytest
import rospy
from sensor_msgs.msg import CameraInfo, Image

from x30_perception.aruco_detector_node import ArucoDetectorNode
from x30_utils.testing.harness import connected, delete_params, wait_for

cv2 = pytest.importorskip('cv2')

from aruco_scene import K, render, yaw_rotation  # noqa: E402


def test_a_marker_in_view_becomes_a_tf_frame_under_the_camera(ros):
    node = ArucoDetectorNode(param_ns='/aruco_test')
    info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1, latch=True)
    image_pub = rospy.Publisher('image', Image, queue_size=1)
    try:
        info = CameraInfo(width=640, height=480, K=K.ravel().tolist(), D=[0.0] * 5)
        info.header.frame_id = 'camera_optical'
        info_pub.publish(info)
        assert connected(info_pub, image_pub)
        assert wait_for(lambda: node.camera is not None, 5.0)
        t = np.array([0.10, -0.05, 1.2])
        gray = render(4, yaw_rotation(np.radians(20.0)), t)
        image = Image(height=480, width=640, encoding='mono8', step=640, data=gray.tobytes())
        image.header.frame_id = 'camera_optical'

        def seen():
            image.header.stamp = rospy.Time.now()
            image_pub.publish(image)
            return node.tf.lookup('camera_optical', 'charging_dock') is not None

        assert wait_for(seen, 5.0, 0.1)
        result = node.tf.lookup('camera_optical', 'charging_dock')
        assert result.matrix[:3, 3] == pytest.approx(t, abs=0.02)  # the unit test checks the pose
    finally:
        node.destroy_node()
        info_pub.unregister()
        image_pub.unregister()
        delete_params('/aruco_test')
