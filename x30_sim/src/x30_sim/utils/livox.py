"""The X30's /livox/lidar (livox_ros_driver2/CustomMsg) as bytes: no Livox package needed."""

from __future__ import annotations

import struct

import numpy as np
import rospy

TYPE = 'livox_ros_driver2/CustomMsg'
MD5 = 'e4d6829bdfe657cb6c21a746c86b21a6'  # livox_ros_driver/CustomMsg has the same
REFLECTIVITY = 100
POINT = np.dtype(
    [
        ('offset_time', '<u4'),
        ('x', '<f4'),
        ('y', '<f4'),
        ('z', '<f4'),
        ('reflectivity', 'u1'),
        ('tag', 'u1'),
        ('line', 'u1'),
    ]
)  # packed: 19 bytes, as genpy serializes CustomPoint
DEFINITION = """# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase          # The time of first point
uint32 point_num      # Total number of pointclouds
uint8  lidar_id         # Lidar device id number
uint8[3]  rsvd          # Reserved use
CustomPoint[] points    # Pointcloud data

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
#
# sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: livox_ros_driver2/CustomPoint
# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
"""  # the generated definition (trailing spaces of comments dropped)


class CustomMsgBytes(rospy.AnyMsg):
    """A serialized CustomMsg that rospy publishes as it is."""

    _type = TYPE
    _md5sum = MD5
    _full_text = DEFINITION

    def __init__(self, data: bytes = b''):
        super().__init__()
        self._buff = data


def encode(points: np.ndarray, stamp: rospy.Time, frame_id: str, seq: int = 0) -> bytes:
    """Return the CustomMsg bytes of (N, 3) points taken at stamp (no motion within the scan)."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    name = frame_id.encode()
    head = struct.pack('<IIII', seq & 0xFFFFFFFF, stamp.secs, stamp.nsecs, len(name)) + name
    count = len(points)
    head += struct.pack('<QIB3xI', stamp.to_nsec(), count, 0, count)
    body = np.zeros(count, dtype=POINT)
    body['x'], body['y'], body['z'] = points[:, 0], points[:, 1], points[:, 2]
    body['reflectivity'] = REFLECTIVITY
    return head + body.tobytes()
