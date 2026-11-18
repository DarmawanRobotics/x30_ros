"""/livox/lidar bytes: what x30_perception (and FAST-LIO, the same md5) read from the X30."""

from __future__ import annotations

import io

import numpy as np
import pytest
import rospy

from x30_perception.core.livox import TYPES, parse_custom_msg
from x30_sim.utils.livox import MD5, POINT, TYPE, CustomMsgBytes, encode


def test_perception_reads_the_points_stamp_and_frame():
    points = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, -1.0]], dtype=np.float32)
    scan = parse_custom_msg(encode(points, rospy.Time(12, 34), 'livox_frame', seq=7))
    assert (scan.sec, scan.nanosec, scan.frame_id) == (12, 34, 'livox_frame')
    assert np.array_equal(scan.xyz, points) and not scan.tags.any()


def test_an_empty_scan_is_valid():
    scan = parse_custom_msg(encode(np.empty((0, 3)), rospy.Time(1, 0), 'livox_frame'))
    assert scan.xyz.shape == (0, 3)


def test_the_layout_is_genpy_s():
    """Header, timebase (ns), point_num, lidar_id, 3 reserved, the array: 19 bytes a point."""
    data = encode(np.zeros((3, 3)), rospy.Time(2, 5), 'f', seq=1)
    assert POINT.itemsize == 19 and len(data) == 16 + 1 + 20 + 3 * 19
    assert int.from_bytes(data[17:25], 'little') == 2_000_000_005


def test_rospy_publishes_the_bytes_under_the_livox_type():
    message = CustomMsgBytes(b'abc')
    buffer = io.BytesIO()
    message.serialize(buffer)
    assert buffer.getvalue() == b'abc'
    assert CustomMsgBytes._type == TYPE and TYPE in TYPES
    assert CustomMsgBytes._md5sum == MD5 == 'e4d6829bdfe657cb6c21a746c86b21a6'
    assert CustomMsgBytes._full_text.splitlines()[2].startswith('std_msgs/Header header')
    with pytest.raises(ValueError):
        parse_custom_msg(b'abc')
