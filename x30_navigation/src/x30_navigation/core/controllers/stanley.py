"""Stanley steering adapted to a unicycle, with curvature feedforward."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
)
from x30_utils.config import param


@dataclass(frozen=True)
class StanleyConfig:
    """Parameters of the Stanley controller."""

    k_cross_track: float = param(1.0, min=0.0, max=20.0, doc='cross-track gain')
    k_soft: float = param(
        0.2, min=0.01, max=5.0, unit='m/s', doc='softening so low speeds do not over-steer'
    )
    k_heading: float = param(1.5, min=0.0, max=20.0, unit='1/s', doc='steering to yaw rate gain')


@CONTROLLERS.register('stanley')
class StanleyController(PathController):
    """Yaw rate v k + k_h (-(e_yaw + atan(k e / (v + k_soft)))) toward the offset route."""

    Config = StanleyConfig

    def compute(self, request: TrackRequest) -> Command:
        """Return forward speed and the Stanley yaw rate."""
        cfg, route, v = self.config, request.route, request.speed
        lateral = route.lateral(request.pose, request.s) - request.offset
        heading = route.heading_error(request.pose, request.s)
        steer = -(heading + math.atan2(cfg.k_cross_track * lateral, v + cfg.k_soft))
        wz = v * route.curvature_at(request.s) + cfg.k_heading * steer
        self.debug = {'nearest': tuple(route.shifted_point(request.s, request.offset))}
        return Command(v, 0.0, self.clamp_wz(wz))
