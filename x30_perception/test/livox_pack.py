"""Hand packer of ROS 1 Livox CustomMsg bytes, shared by the unit and node tests."""

from __future__ import annotations

import struct

import numpy as np


def pack(
    points,
    frame_id: str = 'livox_frame',
    sec: int = 12,
    nanosec: int = 34,
    tags=None,
    seq: int = 0,
) -> bytes:
    """Serialize a CustomMsg by hand, the way genpy (rospy, roscpp) does."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    tags = np.zeros(len(points), dtype=np.uint8) if tags is None else np.asarray(tags, np.uint8)
    if len(tags) != len(points):
        raise ValueError(f'{len(tags)} tags for {len(points)} points')
    name = frame_id.encode()
    buf = bytearray(struct.pack('<IIII', seq, sec, nanosec, len(name)) + name)
    buf += struct.pack('<QIB3xI', 42, len(points), 1, len(points))
    for i, ((x, y, z), tag) in enumerate(zip(points, tags)):
        buf += struct.pack('<IfffBBB', i, x, y, z, 5, int(tag), 1)
    return bytes(buf)
