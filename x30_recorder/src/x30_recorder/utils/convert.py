"""Messages -> what the recorder writes: temperatures, poses, camera positions, plain dicts."""

from __future__ import annotations

import math

import numpy as np

from x30_utils.io import image_temperatures
from x30_utils.ros.convert import msg_yaw
from x30_utils.ros.time import to_sec


def stamp_of(msg, now: float) -> float:
    """Return a message's stamp (s), or now when it has none."""
    stamp = to_sec(msg.header.stamp)
    return stamp if stamp > 0.0 else now


def temperatures(msg) -> np.ndarray:
    """Return the degC of a 32FC1 sensor_msgs/Image (ValueError for another one)."""
    if msg.encoding != '32FC1':
        raise ValueError(f'a thermal frame is 32FC1, not {msg.encoding!r}')
    return image_temperatures(msg.height, msg.width, msg.step, msg.data, bool(msg.is_bigendian))


def planar_pose(msg) -> tuple[float, float, float]:
    """Return (x, y, yaw) of a geometry_msgs/PoseWithCovarianceStamped."""
    pose = msg.pose.pose
    return float(pose.position.x), float(pose.position.y), msg_yaw(pose.orientation)


def ptz(msg) -> tuple[float, float, float]:
    """Return (pan, tilt, zoom) of a hikmicro_msgs/PtzState (NaN while not connected)."""
    if not msg.connected:
        return math.nan, math.nan, math.nan
    return float(msg.pan), float(msg.tilt), float(msg.zoom)


def as_dict(msg) -> dict:
    """Return a message's fields as plain values (its header as stamp and frame_id)."""
    out = {}
    for name in msg.__slots__:
        value = getattr(msg, name)
        if name == 'header':
            out['stamp'], out['frame_id'] = to_sec(value.stamp), value.frame_id
        elif isinstance(value, (bytes, bytearray)):
            out[name] = len(value)  # raw bytes go to the .npz chunks, not the text log
        elif isinstance(value, (list, tuple)):
            out[name] = list(value)
        else:
            out[name] = value
    return out
