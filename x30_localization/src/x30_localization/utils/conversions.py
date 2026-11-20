"""Messages <-> core types of the localization node."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

from x30_localization.core.localizer import Status
from x30_msgs.msg import LocalizationStatus
from x30_utils.math import Pose2D
from x30_utils.ros.convert import (
    planar_covariance,
    pose2d_to_pose,
    pose_to_pose2d,
)
from x30_utils.ros.time import to_sec

PLANAR = [0, 1, 5]


def status_msg(status: Status, stamp, frame: str) -> LocalizationStatus:
    """Return the LocalizationStatus message of a status."""
    msg = LocalizationStatus()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.state = status.state.value
    msg.source = status.source
    msg.sigma_xy = _finite_or_inf(status.sigma_xy)
    msg.sigma_yaw = _finite_or_inf(status.sigma_yaw)
    msg.since_correction = _finite_or_inf(status.since_correction)
    msg.message = status.message
    return msg


def _finite_or_inf(value: float) -> float:
    return float(value) if math.isfinite(value) else math.inf


def pose_msg(pose: Pose2D, cov3: np.ndarray, stamp, frame: str) -> PoseWithCovarianceStamped:
    """Return map->base as PoseWithCovarianceStamped (planar covariance)."""
    msg = PoseWithCovarianceStamped()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.pose.pose = pose2d_to_pose(pose)
    msg.pose.covariance = planar_covariance(cov3)
    return msg


def initial_pose(msg: PoseWithCovarianceStamped) -> tuple[Pose2D, np.ndarray, float]:
    """Return (pose, 3x3 covariance, stamp) of an RViz 2D Pose Estimate."""
    cov6 = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
    return pose_to_pose2d(msg.pose.pose), cov6[np.ix_(PLANAR, PLANAR)], to_sec(msg.header.stamp)


def imu_vectors(msg) -> tuple[float, np.ndarray, np.ndarray]:
    """Return (stamp, linear acceleration, angular velocity) of a sensor_msgs/Imu."""
    a, w = msg.linear_acceleration, msg.angular_velocity
    return to_sec(msg.header.stamp), np.array([a.x, a.y, a.z]), np.array([w.x, w.y, w.z])
