"""Odometry motion counted in steps, so standing still never grows the uncertainty."""

from __future__ import annotations

import math

from x30_utils.math import Pose2D


class MotionCounter:
    """Distance (m) and rotation (rad) of odom->base, counted once a step is completed."""

    def __init__(self, step_m: float, step_rad: float):
        self.step_m, self.step_rad = step_m, step_rad
        self.anchor: Pose2D | None = None

    def reset(self):
        """Start counting from the next pose."""
        self.anchor = None

    def update(self, odom_base: Pose2D) -> tuple[float, float]:
        """Return (distance, rotation) since the last completed step, or (0, 0)."""
        if self.anchor is None:
            self.anchor = odom_base
            return 0.0, 0.0
        delta = self.anchor.between(odom_base)
        distance = math.hypot(delta.x, delta.y)
        if distance < self.step_m and abs(delta.yaw) < self.step_rad:
            return 0.0, 0.0
        self.anchor = odom_base
        return distance, abs(delta.yaw)
