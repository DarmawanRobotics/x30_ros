"""Braitenberg-style avoidance: danger left and right steers, danger ahead slows."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.avoidance.base import POLICIES, AvoidanceOut, AvoidancePolicy
from x30_utils.config import param


@dataclass(frozen=True)
class BraitenbergConfig:
    """Parameters of the Braitenberg policy."""

    steering_gain: float = param(1.0, min=0.0, max=10.0, unit='rad', doc='turn per unit danger')
    velocity_gain: float = param(1.0, min=0.0, max=10.0, doc='slow-down per unit danger')
    steering_zone_center_deg: float = param(45.0, min=0.0, max=180.0, unit='deg')
    steering_zone_width_deg: float = param(60.0, min=1.0, max=360.0, unit='deg')
    velocity_zone_center_deg: float = param(60.0, min=0.0, max=180.0, unit='deg')
    velocity_zone_width_deg: float = param(90.0, min=1.0, max=360.0, unit='deg')
    following_zone_width_deg: float = param(30.0, min=1.0, max=360.0, unit='deg')


def tent(angles: np.ndarray, center: float, width: float) -> np.ndarray:
    """Return a triangular weight: 1 at center, 0 at center +- width / 2."""
    d = np.abs(np.arctan2(np.sin(angles - center), np.cos(angles - center)))
    return np.clip(1.0 - d / (width / 2.0), 0.0, 1.0)


def zone_danger(danger: np.ndarray, weight: np.ndarray) -> float:
    """Return the weighted mean danger of a zone (0 when the zone is empty)."""
    total = float(weight.sum())
    return float((weight * danger).sum() / total) if total > 1e-6 else 0.0


@POLICIES.register('braitenberg')
class BraitenbergAvoidance(AvoidancePolicy):
    """Steer away from the side with more danger, slow down for danger ahead or beside."""

    Config = BraitenbergConfig

    def compute(self, angles, ranges, scan) -> AvoidanceOut:
        """Return the raw output."""
        cfg = self.config
        danger = self.danger(ranges)
        rad = math.radians
        steer_c, steer_w = rad(cfg.steering_zone_center_deg), rad(cfg.steering_zone_width_deg)
        vel_c, vel_w = rad(cfg.velocity_zone_center_deg), rad(cfg.velocity_zone_width_deg)
        left = zone_danger(danger, tent(angles, steer_c, steer_w))
        right = zone_danger(danger, tent(angles, -steer_c, steer_w))
        side = max(
            zone_danger(danger, tent(angles, vel_c, vel_w)),
            zone_danger(danger, tent(angles, -vel_c, vel_w)),
        )
        ahead = zone_danger(danger, tent(angles, 0.0, rad(cfg.following_zone_width_deg)))
        scale = max(0.0, 1.0 - cfg.velocity_gain * side) * (1.0 - ahead)
        offset = cfg.steering_gain * (right - left)
        return AvoidanceOut(heading_offset=offset, velocity_scale=scale)
