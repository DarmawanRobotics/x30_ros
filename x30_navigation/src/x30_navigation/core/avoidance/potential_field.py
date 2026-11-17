"""Artificial potential field: every obstacle inside safe_distance pushes the robot away."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.avoidance.base import POLICIES, AvoidanceOut, AvoidancePolicy
from x30_utils.config import param

MIN_RANGE = 0.05


@dataclass(frozen=True)
class PotentialFieldConfig:
    """Parameters of the potential field policy."""

    repulsion_gain: float = param(0.5, min=0.0, max=10.0, doc='strength of the repulsion')
    steering_gain: float = param(0.8, min=0.0, max=5.0, unit='rad', doc='turn per atan(force)')
    lateral_gain: float = param(0.3, min=0.0, max=5.0, doc='sideways m/s per unit force')
    max_lateral: float = param(0.2, min=0.0, max=2.0, unit='m/s', doc='cap on the sideways push')
    velocity_gain: float = param(1.0, min=0.0, max=10.0, doc='slow-down per unit frontal danger')


@POLICIES.register('potential_field')
class PotentialFieldAvoidance(AvoidancePolicy):
    """Sum the repulsions; their sideways part steers and pushes, danger ahead slows."""

    Config = PotentialFieldConfig

    def compute(self, angles, ranges, scan) -> AvoidanceOut:
        """Return the raw output."""
        cfg, safe = self.config, self.shared.safe_distance
        near = ranges < safe
        r = np.maximum(ranges[near], MIN_RANGE)
        magnitude = cfg.repulsion_gain * (1.0 / r - 1.0 / safe) / (r * r)
        fy = -float((magnitude * np.sin(angles[near])).sum())
        cone = self.in_cone(angles)
        frontal = float(self.danger(ranges)[cone].max()) if cone.any() else 0.0
        lateral = max(-cfg.max_lateral, min(cfg.max_lateral, cfg.lateral_gain * fy))
        return AvoidanceOut(
            heading_offset=cfg.steering_gain * math.atan(fy),
            lateral_bias=lateral,
            velocity_scale=max(0.0, 1.0 - cfg.velocity_gain * frontal),
        )
