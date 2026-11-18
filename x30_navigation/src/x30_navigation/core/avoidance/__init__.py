"""Reactive avoidance policies, registered in POLICIES (entry point group 'x30.avoidance')."""

from __future__ import annotations

from x30_navigation.core.avoidance.base import (
    POLICIES,
    AvoidanceOut,
    AvoidancePolicy,
    AvoidanceShared,
)
from x30_navigation.core.avoidance.braitenberg import BraitenbergAvoidance
from x30_navigation.core.avoidance.follow_gap import FollowGapAvoidance
from x30_navigation.core.avoidance.potential_field import PotentialFieldAvoidance
from x30_navigation.core.avoidance.vfh import VfhAvoidance

__all__ = [
    'POLICIES',
    'AvoidanceOut',
    'AvoidancePolicy',
    'AvoidanceShared',
    'BraitenbergAvoidance',
    'FollowGapAvoidance',
    'PotentialFieldAvoidance',
    'VfhAvoidance',
]
