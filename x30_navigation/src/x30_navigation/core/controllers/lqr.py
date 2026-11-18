"""Discrete LQR on the path error [lateral, heading] with curvature feedforward."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
)
from x30_utils.config import param


@dataclass(frozen=True)
class LqrConfig:
    """Parameters of the LQR controller."""

    q_lateral: float = param(1.0, min=0.0, max=100.0, doc='weight on the cross-track error')
    q_heading: float = param(0.5, min=0.0, max=100.0, doc='weight on the heading error')
    r_yaw_rate: float = param(
        0.5, min=0.001, max=100.0, doc='weight on the yaw rate (higher = gentler)'
    )


@CONTROLLERS.register('lqr')
class LqrController(PathController):
    """Yaw rate v k - K [e_lat, e_yaw], with K from the Riccati equation per speed."""

    Config = LqrConfig

    def __init__(self, config=None, limits=None):
        super().__init__(config, limits)
        self.gains: dict[float, np.ndarray] = {}

    def gain(self, speed: float) -> np.ndarray:
        """Return the LQR gain for a forward speed, cached on a 5 cm/s grid."""
        key = round(max(speed, 0.05) / 0.05) * 0.05
        if key not in self.gains:
            dt = self.limits.period_sec
            a = np.array([[1.0, key * dt], [0.0, 1.0]])
            b = np.array([[0.0], [dt]])
            q = np.diag([self.config.q_lateral, self.config.q_heading])
            r = np.array([[self.config.r_yaw_rate]])
            p = q.copy()
            k = np.zeros((1, 2))
            for _ in range(500):
                btp = b.T @ p
                k = np.linalg.solve(r + btp @ b, btp @ a)
                p_next = q + a.T @ p @ (a - b @ k)
                if np.max(np.abs(p_next - p)) < 1e-9:
                    break
                p = p_next
            self.gains[key] = k
        return self.gains[key]

    def compute(self, request: TrackRequest) -> Command:
        """Return forward speed and the LQR yaw rate toward the offset route."""
        route, v = request.route, request.speed
        state = np.array(
            [
                route.lateral(request.pose, request.s) - request.offset,
                route.heading_error(request.pose, request.s),
            ]
        )
        feedback = float(-(self.gain(v) @ state)[0])
        self.debug = {'nearest': tuple(route.shifted_point(request.s, request.offset))}
        return Command(v, 0.0, self.clamp_wz(v * route.curvature_at(request.s) + feedback))
