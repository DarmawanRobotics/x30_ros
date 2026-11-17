"""Model predictive path integral control of the yaw rate, with obstacle cost."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    PathController,
    TrackRequest,
    carrot,
)
from x30_utils.config import param
from x30_utils.math import default_rng


@dataclass(frozen=True)
class MppiConfig:
    """Parameters of MPPI."""

    horizon_steps: int = param(15, min=2, max=100, doc='steps per rollout')
    step_sec: float = param(0.1, min=0.01, max=1.0, unit='s', doc='rollout step')
    samples: int = param(200, min=10, max=5000, doc='rollouts per tick')
    yaw_rate_std: float = param(1.0, min=0.01, max=5.0, unit='rad/s', doc='sampling noise')
    temperature: float = param(
        0.05, min=0.001, max=100.0, doc='softmax sharpness of the normalized cost'
    )
    window: float = param(4.0, min=0.5, max=20.0, unit='m', doc='route ahead in the cost')
    obstacle_weight: float = param(5.0, min=0.0, max=1000.0, doc='cost per m of intrusion')
    clearance: float = param(
        0.5, min=0.0, max=5.0, unit='m', doc='rollouts closer to an obstacle pay for it'
    )
    seed: int = param(0, min=0, max=2**31 - 1, live=False, doc='random seed (reproducible)')


@CONTROLLERS.register('mppi')
class MppiController(PathController):
    """Sample yaw-rate sequences around the previous plan, roll out a unicycle, average by cost."""

    Config = MppiConfig

    def __init__(self, config=None, limits=None):
        super().__init__(config, limits)
        self.rng = default_rng(self.config.seed)
        self.plan = np.zeros(self.config.horizon_steps)

    def reset(self):
        """Forget the warm start."""
        super().reset()
        self.plan = np.zeros(self.config.horizon_steps)

    def compute(self, request: TrackRequest) -> Command:
        """Return forward speed and the first yaw rate of the cost-weighted sequence."""
        cfg = self.config
        speed = max(request.speed, 0.05)
        samples = self.rng.normal(self.plan, cfg.yaw_rate_std, (cfg.samples, cfg.horizon_steps))
        samples[0] = self.plan
        samples = np.clip(samples, -self.limits.max_wz, self.limits.max_wz)
        xs, ys = self.rollout(request, samples, speed)
        cost = self.cost(request, xs, ys, speed) + 0.01 * np.mean(samples**2, axis=1)
        weights = np.exp(-(cost - cost.min()) / cfg.temperature)
        weights /= weights.sum()
        best = weights @ samples
        self.plan = np.append(best[1:], best[-1])
        k = int(np.argmax(weights))
        self.debug = {'rollout': np.column_stack((xs[k], ys[k]))}
        return Command(request.speed, 0.0, self.clamp_wz(float(best[0])))

    def rollout(self, request: TrackRequest, samples: np.ndarray, speed: float):
        """Return the x and y of every rollout step."""
        dt, pose = self.config.step_sec, request.pose
        yaw = pose.yaw + np.cumsum(samples, axis=1) * dt
        xs = pose.x + np.cumsum(speed * np.cos(yaw) * dt, axis=1)
        ys = pose.y + np.cumsum(speed * np.sin(yaw) * dt, axis=1)
        return xs, ys

    def cost(self, request: TrackRequest, xs: np.ndarray, ys: np.ndarray, speed: float):
        """Return the cost of every rollout."""
        cfg = self.config
        horizon = speed * cfg.step_sec * cfg.horizon_steps
        ahead = np.linspace(request.s, request.s + max(cfg.window, horizon), 40)
        line = np.array([request.route.shifted_point(s, request.offset) for s in ahead])
        dx = xs[:, :, None] - line[None, None, :, 0]
        dy = ys[:, :, None] - line[None, None, :, 1]
        tracking = (dx * dx + dy * dy).min(axis=2).mean(axis=1)
        target = carrot(request, horizon)
        terminal = (xs[:, -1] - target[0]) ** 2 + (ys[:, -1] - target[1]) ** 2
        return tracking + terminal + cfg.obstacle_weight * self.intrusion(request, xs, ys)

    def intrusion(self, request: TrackRequest, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Return the deepest intrusion of every rollout into the obstacles' clearance (m)."""
        points = np.asarray(request.obstacles, dtype=float).reshape(-1, 2)
        if points.size == 0 or self.config.obstacle_weight <= 0.0:
            return np.zeros(len(xs))
        c, s = np.cos(request.pose.yaw), np.sin(request.pose.yaw)
        world = points @ np.array([[c, s], [-s, c]]) + (request.pose.x, request.pose.y)
        dx = xs[:, :, None] - world[None, None, :, 0]
        dy = ys[:, :, None] - world[None, None, :, 1]
        nearest = np.sqrt((dx * dx + dy * dy).min(axis=2))
        return np.clip(self.config.clearance - nearest, 0.0, None).max(axis=1)
