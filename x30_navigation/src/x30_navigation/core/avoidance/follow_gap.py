"""Follow the gap: inflate the obstacles by the body, aim at the centre of the widest gap."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.avoidance.base import POLICIES, AvoidanceOut, AvoidancePolicy
from x30_navigation.core.avoidance.vfh import runs
from x30_utils.config import param


@dataclass(frozen=True)
class FollowGapConfig:
    """Parameters of the follow-the-gap policy."""

    bubble_radius: float = param(
        0.45,
        min=0.0,
        max=5.0,
        unit='m',
        doc='safety bubble around the nearest obstacle (at least half_width + body_margin)',
    )
    steering_gain: float = param(1.0, min=0.0, max=5.0, doc='fraction of the gap heading taken')
    velocity_gain: float = param(1.0, min=0.0, max=10.0, doc='slow-down per unit frontal danger')


@POLICIES.register('follow_gap')
class FollowGapAvoidance(AvoidancePolicy):
    """Only while the frontal cone holds an obstacle, steer to the widest gap's centre."""

    Config = FollowGapConfig

    def compute(self, angles, ranges, scan) -> AvoidanceOut:
        """Return the raw output."""
        cfg, safe = self.config, self.shared.safe_distance
        cone = self.in_cone(angles)
        frontal = float(ranges[cone].min()) if cone.any() else scan.range_max
        scale = max(0.0, 1.0 - cfg.velocity_gain * max(0.0, 1.0 - frontal / safe))
        if frontal >= safe:
            return AvoidanceOut(velocity_scale=scale)
        obstacle = ranges < safe
        nearest = int(np.argmin(ranges))
        half = math.atan2(cfg.bubble_radius, max(float(ranges[nearest]), 1e-3))
        bubble = np.abs(
            np.arctan2(np.sin(angles - angles[nearest]), np.cos(angles - angles[nearest]))
        )
        free = ~obstacle & (bubble > half)
        free &= ~self.blocked_headings(angles, ranges, obstacle, scan.angle_increment)
        gaps = runs(free)
        if not gaps:
            return AvoidanceOut(velocity_scale=0.0)
        start, end = max(gaps, key=lambda g: g[1] - g[0])
        limit = math.radians(self.shared.max_heading_offset_deg)
        lo, hi = max(float(angles[start]), -limit), min(float(angles[end]), limit)
        if lo > hi:
            return AvoidanceOut(velocity_scale=0.0)
        heading = min(hi, max(lo, 0.5 * float(angles[start] + angles[end])))
        return AvoidanceOut(heading_offset=cfg.steering_gain * heading, velocity_scale=scale)
