"""Quaternions in ROS order (x, y, z, w)."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

# typing.Tuple: a runtime alias, subscripting tuple needs Python 3.9.
Quaternion = Tuple[float, float, float, float]

IDENTITY: Quaternion = (0.0, 0.0, 0.0, 1.0)


def quat_from_yaw(yaw: float) -> Quaternion:
    """Return the quaternion of a rotation about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw (rotation about z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Return the quaternion of fixed-axis roll, pitch, yaw (REP-103)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Return roll, pitch, yaw of a quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch, yaw_from_quat(x, y, z, w)


def quat_normalize(q) -> Quaternion:
    """Return q with unit norm (identity for a zero quaternion)."""
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12 or not math.isfinite(n):
        return IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_multiply(a, b) -> Quaternion:
    """Return the Hamilton product a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_matrix(q) -> np.ndarray:
    """Return the 3x3 rotation matrix of a quaternion."""
    x, y, z, w = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_matrix(r: np.ndarray) -> Quaternion:
    """Return the quaternion of a 3x3 rotation matrix (Shepperd's method)."""
    m = np.asarray(r, dtype=float)[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    q = quat_normalize(q)
    return q if q[3] >= 0.0 else (-q[0], -q[1], -q[2], -q[3])
