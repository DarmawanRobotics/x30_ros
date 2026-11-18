"""Drift model of the simulated odometry: scale, yaw bias per metre, noise with travel."""

from __future__ import annotations

import math

import numpy as np

from x30_sim.core.config import OdometryNoiseConfig
from x30_utils.math import Pose2D, compose, inverse


class DriftingOdometry:
    """odom -> base_link from the true motion with scale error, yaw bias and random walk."""

    def __init__(self, config: OdometryNoiseConfig, rng: np.random.Generator, true_pose: Pose2D):
        self.config = config
        self.rng = rng
        self.pose = Pose2D()  # odom -> base: odom is base_link at the (re)start
        self._last_true = true_pose
        self.distance = 0.0  # m travelled since the (re)start

    def restart(self, true_pose: Pose2D):
        """Start over at the current pose, like a restarted odometry (new origin)."""
        self.pose = Pose2D()
        self._last_true = true_pose
        self.distance = 0.0

    def update(self, true_pose: Pose2D) -> Pose2D:
        """Measure the step from the last true pose to true_pose; return odom -> base."""
        step = self._last_true.between(true_pose)
        self._last_true = true_pose
        dist = math.hypot(step.x, step.y)
        ds = dist + self.config.rotation_weight * abs(step.yaw)
        if ds <= 0.0:
            return self.pose
        self.distance += dist
        self.pose = self.pose @ self.measure(step, dist, ds)
        return self.pose

    def measure(self, step: Pose2D, dist: float, ds: float) -> Pose2D:
        """Return the measured increment of a true step."""
        cfg = self.config
        sigma = np.array([cfg.sigma_x, cfg.sigma_y, math.radians(cfg.sigma_yaw_deg)])
        noise = self.rng.standard_normal(3) * sigma * math.sqrt(ds)
        scale = 1.0 + cfg.scale_error
        bias = math.radians(cfg.yaw_bias_deg_per_m) * dist
        return Pose2D(
            scale * step.x + float(noise[0]),
            scale * step.y + float(noise[1]),
            step.yaw + bias + float(noise[2]),
        )


def lio_transform(odom_base: np.ndarray, base_imu: np.ndarray) -> np.ndarray:
    """Return FAST-LIO's camera_init -> body (4x4) for odom -> base_link."""
    return compose(inverse(base_imu), odom_base, base_imu)
