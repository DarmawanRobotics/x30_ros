"""Motion guard on the 360 deg safety scan, with the braking model of x30_utils.safety."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.config.follower import SafetyConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.separation import separation_speed
from x30_navigation.core.sweep import swept_clearance
from x30_utils.robot import RobotProfile
from x30_utils.safety import rotation_room, stopping_yaw_rate

STALL_SPEED = 0.03


@dataclass(frozen=True)
class Guarded:
    """A guarded command, the free distance along it and whether it was cut to a stop."""

    command: Command
    clearance: float
    stalled: bool


class MotionGuard:
    """Limit commands by the safety scan."""

    def __init__(self, robot: RobotProfile, config: SafetyConfig):
        self.robot = robot
        self.config = config
        self.lateral_margin = config.lateral_margin

    def allowed_speed(self, clearance: float, margin: float | None = None) -> float:
        """Return the highest speed (m/s) for a free distance (m) kept margin (stop_margin)."""
        limits = self.robot.limits
        room = clearance - (self.config.stop_margin if margin is None else margin)
        return separation_speed(
            room, limits.max_decel, limits.reaction_time, self.config.human_speed
        )

    def free(self, ux: float, uy: float, points: np.ndarray) -> tuple[float, float]:
        """Return the free distances (m) along (ux, uy) of the body and of its margin band."""
        fp = self.robot.footprint
        body = swept_clearance(fp, ux, uy, points)
        band = swept_clearance(fp, ux, uy, points, self.lateral_margin)
        return body, band

    def speed_for(self, body: float, band: float) -> float:
        """Return the highest speed (m/s) for the free distances of free()."""
        return min(self.allowed_speed(body), self.allowed_speed(band, self.config.pass_margin))

    def limit(self, command: Command, points: np.ndarray, blocked: bool) -> Guarded:
        """Return the command limited by obstacle points (robot frame) and blocked sectors."""
        if blocked:
            return Guarded(Command(), 0.0, True)
        fp, limits = self.robot.footprint, self.robot.limits
        speed = math.hypot(command.vx, command.vy)
        vx, vy, clearance = command.vx, command.vy, math.inf
        if speed > 1e-6:
            body, clearance = self.free(command.vx / speed, command.vy / speed, points)
            scale = min(1.0, self.speed_for(body, clearance) / speed)
            vx, vy = vx * scale, vy * scale
        wz = command.wz
        if abs(wz) > 1e-6:
            room = rotation_room(fp, points, wz) - math.radians(self.config.yaw_margin_deg)
            allowed = stopping_yaw_rate(room, limits.max_yaw_accel, limits.reaction_time)
            wz = math.copysign(min(abs(wz), allowed), wz)
        stalled = speed > STALL_SPEED and math.hypot(vx, vy) < STALL_SPEED
        return Guarded(Command(vx, vy, wz), clearance, stalled)
