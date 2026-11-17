"""Simplified vector field histogram: turn to the free heading nearest straight ahead."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.avoidance.base import POLICIES, AvoidanceOut, AvoidancePolicy
from x30_utils.config import param


@dataclass(frozen=True)
class VfhConfig:
    """Parameters of the VFH policy."""

    min_gap_width_deg: float = param(
        20.0, min=1.0, max=180.0, unit='deg', doc='narrowest run of free headings taken'
    )
    steering_gain: float = param(1.0, min=0.0, max=5.0, doc='fraction of the gap heading taken')
    velocity_gain: float = param(1.0, min=0.0, max=10.0, doc='slow-down per unit frontal danger')


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Return (start, end) index pairs of the runs of True in a mask."""
    padded = np.concatenate(([False], mask, [False])).astype(np.int8)
    edges = np.flatnonzero(np.diff(padded))
    return [(int(edges[k]), int(edges[k + 1]) - 1) for k in range(0, len(edges), 2)]


@POLICIES.register('vfh')
class VfhAvoidance(AvoidancePolicy):
    """Only when the frontal cone holds an obstacle, take the free heading nearest ahead."""

    Config = VfhConfig

    def compute(self, angles, ranges, scan) -> AvoidanceOut:
        """Return the raw output."""
        cfg = self.config
        cone = self.in_cone(angles)
        obstacle = ranges < self.shared.safe_distance
        frontal = float(self.danger(ranges)[cone].max()) if cone.any() else 0.0
        scale = max(0.0, 1.0 - cfg.velocity_gain * frontal)
        if not np.any(obstacle & cone):
            return AvoidanceOut(velocity_scale=scale)
        inc = abs(scan.angle_increment)
        free = ~self.blocked_headings(angles, ranges, obstacle, inc)
        free &= np.abs(angles) <= math.radians(self.shared.max_heading_offset_deg) + 1e-9
        need = max(1, int(math.radians(cfg.min_gap_width_deg) / inc))
        candidates = [i for a, b in runs(free) if b - a + 1 >= need for i in range(a, b + 1)]
        if not candidates:
            return AvoidanceOut(velocity_scale=0.0)
        best = float(min((angles[i] for i in candidates), key=abs))
        return AvoidanceOut(heading_offset=cfg.steering_gain * best, velocity_scale=scale)
