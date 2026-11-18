"""Rigid transforms in SE(3) as 4x4 homogeneous matrices."""

from __future__ import annotations

from functools import reduce
import math

import numpy as np

from x30_utils.math.quaternion import (
    IDENTITY,
    quat_from_matrix,
    quat_from_yaw,
    quat_to_matrix,
    rpy_from_quat,
)


def make_transform(xyz=(0.0, 0.0, 0.0), quat=IDENTITY) -> np.ndarray:
    """Return the 4x4 matrix of a translation and a quaternion (x, y, z, w)."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(quat)
    m[:3, 3] = xyz
    return m


def planar_transform(x: float, y: float, yaw: float, z: float = 0.0) -> np.ndarray:
    """Return the 4x4 matrix of a planar pose."""
    return make_transform((x, y, z), quat_from_yaw(yaw))


def transform_parts(m: np.ndarray) -> tuple[tuple[float, float, float], tuple]:
    """Return (xyz, quaternion) of a 4x4 matrix."""
    return tuple(float(v) for v in m[:3, 3]), quat_from_matrix(m[:3, :3])


def inverse(m: np.ndarray) -> np.ndarray:
    """Return the inverse of a rigid transform."""
    out = np.eye(4)
    rt = m[:3, :3].T
    out[:3, :3] = rt
    out[:3, 3] = -rt @ m[:3, 3]
    return out


def compose(*matrices: np.ndarray) -> np.ndarray:
    """Return the product of transforms, applied right to left."""
    return reduce(np.matmul, matrices, np.eye(4))


def yaw_of(m: np.ndarray) -> float:
    """Return the yaw of a transform."""
    return math.atan2(m[1, 0], m[0, 0])


def tilt_of(m: np.ndarray) -> tuple[float, float]:
    """Return roll and pitch of a transform."""
    roll, pitch, _ = rpy_from_quat(*quat_from_matrix(m[:3, :3]))
    return roll, pitch


def flatten(m: np.ndarray) -> np.ndarray:
    """Return the planar part (x, y, yaw) of a transform, with z, roll and pitch dropped."""
    return planar_transform(float(m[0, 3]), float(m[1, 3]), yaw_of(m))


def transform_points(m: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a transform to an (N, 3) array of points."""
    points = np.asarray(points, dtype=float)
    return points @ m[:3, :3].T + m[:3, 3]


def rotation_aligning(a, b) -> np.ndarray:
    """Return the 3x3 rotation taking direction a onto direction b (Rodrigues)."""
    a = np.asarray(a, dtype=float) / np.linalg.norm(a)
    b = np.asarray(b, dtype=float) / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -1.0 + 1e-9:
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)
