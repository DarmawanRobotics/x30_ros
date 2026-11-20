"""The last stage before publishing: envelope, smoothing, slew limits, non-finite to zero."""

from __future__ import annotations

import math

from x30_navigation.core.controllers.base import Command
from x30_utils.math import Ema, SlewLimiter, clamp
from x30_utils.robot import RobotProfile


class CommandShaper:
    """Shape a target command: finite, within the limits, filtered and slew limited."""

    def __init__(self, robot: RobotProfile, smoothing: float):
        limits = robot.limits
        self.max_vx, self.max_wz = limits.max_vx, limits.max_wz
        self.max_vy = limits.max_vy if robot.can_strafe else 0.0
        self.vx = SlewLimiter(limits.max_accel, limits.max_decel)
        self.vy = SlewLimiter(limits.max_accel, limits.max_decel)
        self.wz = SlewLimiter(limits.max_yaw_accel, limits.max_yaw_accel)
        self.smooth_vy, self.smooth_wz = Ema(smoothing), Ema(smoothing)
        self.last = Command()

    def envelope(self, target: Command) -> Command:
        """Return the target within the robot's limits (non-finite: 0)."""
        vx, vy = clamp(target.vx, -math.inf, math.inf), clamp(target.vy, -math.inf, math.inf)
        if self.max_vy <= 0.0:
            vy = 0.0
        scale = 1.0
        if abs(vx) > self.max_vx:
            scale = self.max_vx / abs(vx)
        if abs(vy) > self.max_vy and abs(vy) > 0.0:
            scale = min(scale, self.max_vy / abs(vy))
        wz = clamp(target.wz, -self.max_wz, self.max_wz)
        return Command(vx * scale, vy * scale, wz)

    def shape(self, target: Command, dt: float, smooth: bool = True) -> Command:
        """Return the command to publish for target."""
        target = self.envelope(target)
        vy, wz = target.vy, target.wz
        if smooth:
            vy, wz = self.smooth_vy.update(vy), self.smooth_wz.update(wz)
        else:
            self.smooth_vy.value, self.smooth_wz.value = vy, wz
        out = Command(self.vx.step(target.vx, dt), self.vy.step(vy, dt), self.wz.step(wz, dt))
        self.last = self.envelope(out)
        return self.last

    def hold(self, command: Command) -> Command:
        """Return command and go on from it (the guard cut the shaped one)."""
        self.vx.reset(command.vx)
        self.vy.reset(command.vy)
        self.wz.reset(command.wz)
        self.smooth_vy.value, self.smooth_wz.value = command.vy, command.wz
        self.last = command
        return command

    def hold_forward(self):
        """Stop the forward motion at once (no ramp); sideways and turning keep their ramps."""
        self.vx.reset(min(self.vx.value, 0.0))

    def stop(self) -> Command:
        """Return zero at once (no ramp) and forget the filters."""
        for limiter in (self.vx, self.vy, self.wz):
            limiter.reset(0.0)
        self.smooth_vy.reset()
        self.smooth_wz.reset()
        self.last = Command()
        return self.last
