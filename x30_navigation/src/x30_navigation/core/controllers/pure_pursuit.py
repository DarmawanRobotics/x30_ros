"""Pure pursuit: drive on the arc through a carrot on the route."""

from __future__ import annotations

from dataclasses import dataclass

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
    carrot,
    to_body,
)
from x30_utils.config import param


@dataclass(frozen=True)
class PurePursuitConfig:
    """Parameters of pure pursuit."""

    lookahead: float = param(1.0, min=0.1, max=5.0, unit='m', doc='carrot ahead along the route')


@CONTROLLERS.register('pure_pursuit')
class PurePursuitController(PathController):
    """Curvature 2 y / d^2 toward the carrot, at the requested speed."""

    Config = PurePursuitConfig

    def compute(self, request: TrackRequest) -> Command:
        """Return forward speed and the yaw rate of the arc through the carrot."""
        target = carrot(request, self.config.lookahead)
        x, y = to_body(request.pose, target)
        d2 = x * x + y * y
        curvature = 2.0 * y / d2 if d2 > 1e-6 else 0.0
        self.debug = {'carrot': (float(target[0]), float(target[1])), 'curvature': curvature}
        return Command(request.speed, 0.0, self.clamp_wz(request.speed * curvature))
