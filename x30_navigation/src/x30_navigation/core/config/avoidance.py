"""Configuration of obstacle_avoidance_node."""

from __future__ import annotations

from dataclasses import dataclass, is_dataclass

from x30_navigation.core.avoidance.base import AvoidanceShared
from x30_navigation.core.avoidance.braitenberg import BraitenbergConfig
from x30_navigation.core.avoidance.follow_gap import FollowGapConfig
from x30_navigation.core.avoidance.potential_field import PotentialFieldConfig
from x30_navigation.core.avoidance.vfh import VfhConfig
from x30_utils.config import nested, param


@dataclass(frozen=True)
class AvoidanceConfig:
    """obstacle_avoidance_node parameters: the policy, shared distances, per-policy sections."""

    policy: str = param('vfh', doc='avoidance policy (x30.avoidance)')
    shared: AvoidanceShared = nested(AvoidanceShared)
    braitenberg: BraitenbergConfig = nested(BraitenbergConfig)
    vfh: VfhConfig = nested(VfhConfig)
    potential_field: PotentialFieldConfig = nested(PotentialFieldConfig)
    follow_gap: FollowGapConfig = nested(FollowGapConfig)
    markers: bool = param(True, doc='publish RViz markers')

    def validate(self) -> list[str]:
        """Return cross-field problems (the policy must be registered)."""
        from x30_navigation.core.avoidance import POLICIES

        problems = []
        if self.policy not in POLICIES:
            problems.append(f'policy: unknown {self.policy!r}, use {POLICIES.names()}')
        return problems

    def policy_config(self):
        """Return the section of the selected built-in policy (None for a plugin)."""
        section = getattr(self, self.policy, None)
        return section if is_dataclass(section) else None
