"""geometry_msgs <-> numpy transforms and planar poses."""

from __future__ import annotations

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
import numpy as np

from x30_utils.math import (
    Pose2D,
    make_transform,
    quat_from_yaw,
    transform_parts,
    yaw_from_quat,
)


def quat_msg(q) -> Quaternion:
    """Return a Quaternion message from (x, y, z, w)."""
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def yaw_quat_msg(yaw: float) -> Quaternion:
    """Return the Quaternion message of a yaw."""
    return quat_msg(quat_from_yaw(yaw))


def msg_yaw(q: Quaternion) -> float:
    """Return the yaw of a Quaternion message."""
    return yaw_from_quat(q.x, q.y, q.z, q.w)


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Return the 4x4 matrix of a Pose message."""
    p, q = pose.position, pose.orientation
    return make_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def transform_to_matrix(t: Transform) -> np.ndarray:
    """Return the 4x4 matrix of a Transform message."""
    v, q = t.translation, t.rotation
    return make_transform((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))


def matrix_to_pose(m: np.ndarray) -> Pose:
    """Return a Pose message from a 4x4 matrix."""
    xyz, q = transform_parts(m)
    return Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]), orientation=quat_msg(q))


def matrix_to_transform(m: np.ndarray) -> Transform:
    """Return a Transform message from a 4x4 matrix."""
    xyz, q = transform_parts(m)
    return Transform(translation=Vector3(x=xyz[0], y=xyz[1], z=xyz[2]), rotation=quat_msg(q))


def transform_stamped(m: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    """Return a TransformStamped from a 4x4 matrix."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform = matrix_to_transform(m)
    return t


def pose_to_pose2d(pose: Pose) -> Pose2D:
    """Return the planar part of a Pose message."""
    return Pose2D(pose.position.x, pose.position.y, msg_yaw(pose.orientation))


def pose2d_to_pose(p: Pose2D, z: float = 0.0) -> Pose:
    """Return a Pose message from a planar pose."""
    return Pose(position=Point(x=p.x, y=p.y, z=z), orientation=yaw_quat_msg(p.yaw))


def planar_covariance(cov3: np.ndarray) -> list[float]:
    """Return the 6x6 row-major pose covariance of a 3x3 (x, y, yaw) covariance."""
    full = np.zeros((6, 6))
    index = [0, 1, 5]
    full[np.ix_(index, index)] = np.asarray(cov3, dtype=float)
    full[2, 2] = full[3, 3] = full[4, 4] = 1e6
    return full.ravel().tolist()
