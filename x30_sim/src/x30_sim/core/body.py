"""Kinematic robot body: velocity commands, acceleration limits, a stop on contact."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_sim.core.world import World, collides
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class BodyLimits:
    """Acceleration limits and timing of the body."""

    accel: float = 0.5  # m/s^2 speeding up
    decel: float = 0.6  # m/s^2 slowing down
    yaw_accel: float = 1.5  # rad/s^2
    max_dt: float = 0.1  # s, longest integrated step
    omni: bool = True  # False: diff drive, vy is ignored

    @staticmethod
    def from_robot(robot: RobotProfile, max_dt: float) -> BodyLimits:
        """Return the limits of a robot profile."""
        lim = robot.limits
        return BodyLimits(
            lim.max_accel, lim.max_decel, lim.max_yaw_accel, max_dt, robot.can_strafe
        )


@dataclass(frozen=True)
class Twist:
    """Body-frame velocity (m/s, m/s, rad/s)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self) -> bool:
        """Return True when every component is zero."""
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


def _toward(value: float, target: float, accel: float, decel: float, dt: float) -> float:
    """Move value toward target: accel while the magnitude grows, decel while it shrinks."""
    speeding_up = abs(target) > abs(value) and target * value >= 0.0
    step = (accel if speeding_up else decel) * dt
    return value + max(-step, min(step, target - value))


def integrate(pose: Pose2D, twist: Twist, dt: float) -> Pose2D:
    """Return the pose after moving with a constant body twist for dt (exact SE(2) arc)."""
    dyaw = twist.wz * dt
    if abs(dyaw) < 1e-9:
        dx, dy = twist.vx * dt, twist.vy * dt
    else:
        s, c = math.sin(dyaw), math.cos(dyaw)
        a, b = s / twist.wz, (1.0 - c) / twist.wz
        dx, dy = a * twist.vx - b * twist.vy, b * twist.vx + a * twist.vy
    return pose @ Pose2D(dx, dy, dyaw)


class Body:
    """Planar robot body (omni or diff) in a world, stepped with measured time."""

    def __init__(
        self,
        pose: Pose2D,
        limits: BodyLimits,
        footprint: tuple[tuple[float, float], tuple[float, float]] | None = None,
        world: World | None = None,
    ):
        self.pose = pose
        self.twist = Twist()
        self.accel = Twist()  # d twist / dt of the last step (the IMU's linear acceleration)
        self.limits = limits
        self.footprint = footprint  # ((x_min, y_min), (x_max, y_max)) in base_link
        self.world = world
        self.collided = False
        self.last_time: float | None = None

    @staticmethod
    def footprint_box(robot: RobotProfile) -> tuple[tuple[float, float], tuple[float, float]]:
        """Return the footprint box of a robot profile."""
        f = robot.footprint
        return (-f.rear, -f.half_width), (f.front, f.half_width)

    def step(self, now: float, command: Twist, extra_circles: np.ndarray | None = None) -> float:
        """Advance to time now (s) under command; return the integrated dt."""
        if self.last_time is None or now <= self.last_time:
            self.last_time = now if self.last_time is None else self.last_time
            return 0.0
        dt = min(now - self.last_time, self.limits.max_dt)
        self.last_time = now
        lim = self.limits
        target = command if lim.omni else Twist(command.vx, 0.0, command.wz)
        before = self.twist
        self.twist = Twist(
            _toward(before.vx, target.vx, lim.accel, lim.decel, dt),
            _toward(before.vy, target.vy, lim.accel, lim.decel, dt),
            _toward(before.wz, target.wz, lim.yaw_accel, lim.yaw_accel, dt),
        )
        candidate = integrate(self.pose, self.twist, dt)
        if self._blocked(candidate, extra_circles):
            self.collided = True
            self.twist = Twist()
        else:
            self.pose = candidate
        self.accel = Twist(
            (self.twist.vx - before.vx) / dt,
            (self.twist.vy - before.vy) / dt,
            (self.twist.wz - before.wz) / dt,
        )
        return dt

    def _blocked(self, candidate: Pose2D, extra) -> bool:
        """Return True when the move enters an obstacle (moving out of one is allowed)."""
        if self.world is None or self.footprint is None:
            return False
        lo, hi = self.footprint
        if not collides(self.world, candidate, lo, hi, extra):
            return False
        if collides(self.world, self.pose, lo, hi, extra):
            self.collided = True
            return False
        return True
