"""PID on the heading error toward a carrot, with anti-windup."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
    carrot,
)
from x30_utils.config import param
from x30_utils.math import wrap_angle


@dataclass(frozen=True)
class PidConfig:
    """Parameters of the PID controller."""

    lookahead: float = param(1.0, min=0.1, max=5.0, unit='m', doc='carrot ahead along the route')
    kp: float = param(1.5, min=0.0, max=20.0, unit='1/s', doc='proportional gain')
    ki: float = param(0.0, min=0.0, max=20.0, unit='1/s^2', doc='integral gain')
    kd: float = param(0.2, min=0.0, max=20.0, doc='derivative gain')
    integral_limit: float = param(
        0.5, min=0.0, max=10.0, unit='rad s', doc='anti-windup clamp of the error integral'
    )


@CONTROLLERS.register('pid')
class PidController(PathController):
    """Yaw rate from a PID on the heading error toward the carrot, at the requested speed."""

    Config = PidConfig

    def __init__(self, config=None, limits=None):
        super().__init__(config, limits)
        self.integral = 0.0
        self.last_error: float | None = None

    def reset(self):
        """Clear the integral and the derivative memory."""
        super().reset()
        self.integral = 0.0
        self.last_error = None

    def compute(self, request: TrackRequest) -> Command:
        """Return forward speed and the PID yaw rate."""
        cfg, pose = self.config, request.pose
        target = carrot(request, cfg.lookahead)
        error = wrap_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
        dt = max(request.dt, 1e-3)
        derivative = 0.0 if self.last_error is None else wrap_angle(error - self.last_error) / dt
        self.last_error = error
        unclamped = cfg.kp * error + cfg.ki * self.integral + cfg.kd * derivative
        saturated = abs(unclamped) >= self.limits.max_wz and unclamped * error > 0.0
        if not saturated:
            limit = cfg.integral_limit
            self.integral = max(-limit, min(limit, self.integral + error * dt))
        wz = cfg.kp * error + cfg.ki * self.integral + cfg.kd * derivative
        self.debug = {'carrot': (float(target[0]), float(target[1]))}
        return Command(request.speed, 0.0, self.clamp_wz(wz))
