"""Messages of the simulated robot from core values, and commands / obstacles from messages."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseArray, PoseStamped
from geometry_msgs.msg import Twist as TwistMsg
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Header

from x30_sim.core.body import Twist
from x30_utils.math import Pose2D
from x30_utils.ros.convert import matrix_to_pose, pose2d_to_pose
from x30_utils.ros.time import to_stamp

LEG_POSE_VARIANCE = (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)  # x30_driver's odom covariances
LEG_TWIST_VARIANCE = (0.02, 0.02, 1e6, 1e6, 1e6, 0.05)
NAN = float('nan')


def header(stamp: float, frame: str) -> Header:
    """Return a header at stamp seconds."""
    return Header(stamp=to_stamp(stamp), frame_id=frame)


def twist_from_msg(msg: TwistMsg) -> Twist:
    """Return the planar command of a Twist; non-finite values are zero."""
    values = (msg.linear.x, msg.linear.y, msg.angular.z)
    return Twist(*(float(v) if math.isfinite(v) else 0.0 for v in values))


def obstacle_positions(msg: PoseArray) -> np.ndarray:
    """Return the finite (N, 2) map positions of a PoseArray."""
    xy = np.array([(p.position.x, p.position.y) for p in msg.poses], dtype=float).reshape(-1, 2)
    return xy[np.isfinite(xy).all(axis=1)]


def odometry_msg(stamp: float, frame: str, child: str, matrix: np.ndarray) -> Odometry:
    """Return FAST-LIO's Odometry: the pose of a 4x4 matrix, no twist (FAST-LIO sends none)."""
    msg = Odometry(header=header(stamp, frame), child_frame_id=child)
    msg.pose.pose = matrix_to_pose(matrix)
    return msg


def _diagonal(variances: tuple) -> list:
    covariance = [0.0] * 36
    for i, variance in enumerate(variances):
        covariance[i * 7] = variance
    return covariance


def leg_odometry_msg(stamp: float, frame: str, child: str, pose: Pose2D, twist: Twist) -> Odometry:
    """Return the driver's odom: leg odometry pose (level), twist in child, its covariances."""
    msg = Odometry(header=header(stamp, frame), child_frame_id=child)
    msg.pose.pose = pose2d_to_pose(pose)
    msg.pose.covariance = _diagonal(LEG_POSE_VARIANCE)
    msg.twist.twist.linear.x, msg.twist.twist.linear.y = twist.vx, twist.vy
    msg.twist.twist.angular.z = twist.wz
    msg.twist.covariance = _diagonal(LEG_TWIST_VARIANCE)
    return msg


def imu_msg(stamp: float, frame: str, accel: np.ndarray, gyro: np.ndarray) -> Imu:
    """Return an Imu (specific force in m/s^2, rates in rad/s; no orientation)."""
    msg = Imu(header=header(stamp, frame))
    msg.orientation_covariance[0] = -1.0
    a, g = msg.linear_acceleration, msg.angular_velocity
    a.x, a.y, a.z = (float(v) for v in accel)
    g.x, g.y, g.z = (float(v) for v in gyro)
    return msg


def pose_stamped(stamp: float, frame: str, pose: Pose2D) -> PoseStamped:
    """Return a planar PoseStamped."""
    return PoseStamped(header=header(stamp, frame), pose=pose2d_to_pose(pose))


def battery_msg(stamp: float, charge: float, charging: bool, report: bool) -> BatteryState:
    """Return a BatteryState of a charge in 0..1; the status only when report (else UNKNOWN)."""
    msg = BatteryState(header=header(stamp, ''))
    msg.percentage = float(charge)
    msg.voltage = msg.current = msg.charge = msg.capacity = msg.design_capacity = NAN
    status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    if report:
        status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if charging:
            status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if charge >= 1.0:
                status = BatteryState.POWER_SUPPLY_STATUS_FULL
    msg.power_supply_status = status
    msg.present = True
    return msg
