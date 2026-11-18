"""The body IMU: specific force (m/s^2) and rates at its origin on the planar, level body."""

from __future__ import annotations

import numpy as np

from x30_sim.core.body import Twist

GRAVITY = 9.80665  # m/s^2
ACCEL_NOISE = 0.02  # m/s^2, 1 sigma per axis
GYRO_NOISE = 0.001  # rad/s, 1 sigma per axis


def imu_sample(
    twist: Twist, accel: Twist, xy: tuple[float, float], rng: np.random.Generator
) -> tuple[np.ndarray, np.ndarray]:
    """Return (accelerometer m/s^2, gyro rad/s) at xy on the body; at rest z reads +g."""
    rx, ry = xy
    w, dw = twist.wz, accel.wz
    ax = accel.vx - w * twist.vy - dw * ry - w * w * rx  # dv/dt + w x v + dw x r + w x (w x r)
    ay = accel.vy + w * twist.vx + dw * rx - w * w * ry
    force = np.array([ax, ay, GRAVITY]) + rng.normal(0.0, ACCEL_NOISE, 3)
    gyro = np.array([0.0, 0.0, w]) + rng.normal(0.0, GYRO_NOISE, 3)
    return force, gyro
