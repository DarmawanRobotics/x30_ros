"""Holonomic path following: x, y and yaw for legged and omnidirectional robots."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
    carrot,
    to_body,
)
from x30_utils.config import param
from x30_utils.math import wrap_angle


@dataclass(frozen=True)
class HolonomicConfig:
    """Parameters of the holonomic controller."""

    lookahead: float = param(
        0.6, min=0.1, max=5.0, unit='m', doc='carrot ahead along the route; longer = gentler'
    )
    heading_lookahead: float = param(
        0.6, min=0.1, max=5.0, unit='m', doc='the heading follows the route this far ahead'
    )
    kp_yaw: float = param(1.5, min=0.0, max=10.0, unit='1/s', doc='yaw rate per rad of error')
    max_lateral_speed: float = param(
        0.25, min=0.0, max=5.0, unit='m/s', doc='sideways speed cap (0: never strafe)'
    )
    max_reverse_speed: float = param(
        0.15, min=0.0, max=5.0, unit='m/s', doc='backward speed cap when the route is behind'
    )
    min_alignment_scale: float = param(
        0.25, min=0.0, max=1.0, doc='translation factor while the heading is far off'
    )


@CONTROLLERS.register('holonomic')
class HolonomicController(PathController):
    """Walk toward a carrot on the (offset) route in any direction, heading along the route."""

    Config = HolonomicConfig
    strafes = True

    def compute(self, request: TrackRequest) -> Command:
        """Return (vx, vy, wz) toward the carrot with the heading along the route."""
        cfg, pose = self.config, request.pose
        target = carrot(request, cfg.lookahead)
        dx, dy = target[0] - pose.x, target[1] - pose.y
        distance = math.hypot(dx, dy)
        heading = request.route.chord_heading(request.s, request.s + cfg.heading_lookahead)
        if self.max_lateral <= 0.0 and distance > 1e-3:
            heading = math.atan2(dy, dx)
        error = wrap_angle(heading - pose.yaw)
        wz = self.clamp_wz(cfg.kp_yaw * error)
        vx = vy = 0.0
        if distance > 1e-3 and request.speed > 0.0:
            speed = min(request.speed, distance)
            vx, vy = to_body(
                pose, (pose.x + speed * dx / distance, pose.y + speed * dy / distance)
            )
            vx, vy = self.limit(vx, vy, error)
        self.debug = {
            'carrot': (float(target[0]), float(target[1])),
            'nearest': tuple(request.route.shifted_point(request.s, request.offset)),
        }
        return Command(vx, vy, wz)

    @property
    def max_lateral(self) -> float:
        """Sideways speed cap: the controller's, never above the robot's."""
        return min(self.config.max_lateral_speed, self.limits.max_vy)

    def limit(self, vx: float, vy: float, heading_error: float) -> tuple[float, float]:
        """Slow down while misaligned, then cap reverse and sideways speed (same direction)."""
        alignment = math.cos(heading_error) if abs(heading_error) < math.pi / 2 else 0.0
        scale = max(self.config.min_alignment_scale, alignment)
        if self.max_lateral <= 0.0:
            return max(0.0, vx) * scale, 0.0
        if vx < 0.0:
            reverse = self.config.max_reverse_speed
            scale = min(scale, reverse / -vx) if reverse > 0.0 else 0.0
        if abs(vy) > 1e-9:
            scale = min(scale, self.max_lateral / abs(vy))
        return vx * scale, vy * scale
