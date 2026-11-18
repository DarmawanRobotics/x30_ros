"""Reactive avoidance policies: one interface, every implementation registered by name."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
from typing import ClassVar

import numpy as np

from x30_navigation.core.avoidance.smoothing import AsymmetricSmoother
from x30_navigation.core.scan import SectorScanData
from x30_utils.config import param
from x30_utils.math import wrap_angles
from x30_utils.plugins import Registry
from x30_utils.robot import RobotProfile

REVERSE_SPEED = 0.02  # m/s: a slower backward command still counts as forward travel


def travel_direction(vx: float, full_circle: bool) -> float:
    """Return the direction of travel (rad): pi while reversing on a 360 deg scan, else 0."""
    return math.pi if full_circle and vx < -REVERSE_SPEED else 0.0


@dataclass(frozen=True)
class AvoidanceOut:
    """How avoidance bends the command (see the module docstring)."""

    heading_offset: float = 0.0
    yaw_rate_bias: float = 0.0
    lateral_bias: float = 0.0
    velocity_scale: float = 1.0
    emergency: bool = False


@dataclass(frozen=True)
class AvoidanceShared:
    """Distances and limits every policy uses."""

    safe_distance: float = param(
        1.5,
        min=0.1,
        max=20.0,
        unit='m',
        doc='obstacles closer than this shape the command (beyond the guard stopping reach)',
    )
    emergency_distance: float = param(
        0.75, min=0.0, max=10.0, unit='m', doc='obstacle in the cone closer than this: stop'
    )
    emergency_cone_deg: float = param(
        60.0, min=1.0, max=360.0, unit='deg', doc='frontal cone of the emergency stop'
    )
    max_heading_offset_deg: float = param(
        60.0, min=0.0, max=180.0, unit='deg', doc='largest turn of the direction of travel'
    )
    smoothing: float = param(
        0.8, min=0.0, max=0.99, doc='0..1 filter on steering and speeding up (never on braking)'
    )
    window_deg: float = param(
        180.0, min=10.0, max=360.0, unit='deg', doc='sectors evaluated around the travel direction'
    )
    body_margin: float = param(
        0.15,
        min=0.0,
        max=2.0,
        unit='m',
        doc='gap kept beside the body on a chosen heading (> the guard lateral_margin)',
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.emergency_distance >= self.safe_distance:
            return ['emergency_distance must be below safe_distance']
        return []


@dataclass(frozen=True)
class NoConfig:
    """A policy without parameters."""


class AvoidancePolicy(ABC):
    """Base class of reactive avoidance policies."""

    Config: ClassVar[type] = NoConfig

    def __init__(
        self,
        shared: AvoidanceShared | None = None,
        config=None,
        robot: RobotProfile | None = None,
    ):
        self.shared = shared or AvoidanceShared()
        self.config = config if config is not None else self.Config()
        self.robot = robot or RobotProfile()
        self.smoother = AsymmetricSmoother(self.shared.smoothing)
        self.direction = 0.0

    def update(self, scan: SectorScanData, direction: float = 0.0) -> AvoidanceOut:
        """Return the smoothed output for one scan; emergency and braking are never delayed."""
        self.direction = float(direction)
        angles, ranges = self.window(scan, self.direction)
        if len(ranges) == 0:  # nothing known around the direction of travel
            return self.smoother.step(AvoidanceOut(velocity_scale=0.0, emergency=True))
        raw = self.compute(angles, ranges, scan)
        limit = math.radians(self.shared.max_heading_offset_deg)
        raw = AvoidanceOut(
            max(-limit, min(limit, raw.heading_offset)),
            raw.yaw_rate_bias,
            raw.lateral_bias * math.cos(self.direction),
            max(0.0, min(1.0, raw.velocity_scale)),
            raw.emergency or self.emergency(angles, ranges),
        )
        return self.smoother.step(raw)

    def window(self, scan: SectorScanData, direction: float) -> tuple[np.ndarray, np.ndarray]:
        """Return relative angles and cleaned ranges of the sectors around a direction, sorted."""
        if len(scan.ranges) == 0:
            return np.empty(0), np.empty(0)
        rel = wrap_angles(scan.angles - direction)
        keep = np.flatnonzero(np.abs(rel) <= math.radians(self.shared.window_deg) / 2.0 + 1e-9)
        order = keep[np.argsort(rel[keep], kind='mergesort')]
        return rel[order], scan.cleaned()[order]

    def emergency(self, angles: np.ndarray, ranges: np.ndarray) -> bool:
        """Return True when something in the frontal cone is closer than emergency_distance."""
        return bool(np.any(self.in_cone(angles) & (ranges < self.shared.emergency_distance)))

    def in_cone(self, angles: np.ndarray) -> np.ndarray:
        """Return the mask of sectors inside the frontal cone."""
        half = math.radians(self.shared.emergency_cone_deg) / 2.0
        return np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < half

    def blocked_headings(
        self, angles: np.ndarray, ranges: np.ndarray, obstacle: np.ndarray, increment: float
    ) -> np.ndarray:
        """Return the mask of headings that pass an obstacle sector closer than body_margin."""
        if not obstacle.any():
            return np.zeros(len(angles), dtype=bool)
        radius = self.half_extent(angles)[:, None] + self.shared.body_margin
        r = np.maximum(ranges[obstacle], 1e-6)[None, :]
        half = np.arcsin(np.clip(radius / r, 0.0, 1.0)) + 0.5 * abs(increment)
        gap = np.abs(wrap_angles(angles[:, None] - angles[obstacle][None, :]))
        return np.any(gap <= half, axis=1)

    def half_extent(self, headings: np.ndarray) -> np.ndarray:
        """Return the half width of the swept path for every heading (relative to travel)."""
        fp = self.robot.footprint
        if not self.robot.can_strafe:
            return np.full(len(headings), fp.half_width)
        body = headings + self.direction
        c, s = np.abs(np.cos(body)), np.abs(np.sin(body))
        length = np.where(np.cos(body) >= 0.0, fp.front, fp.rear)
        return np.minimum(length * s + fp.half_width * c, np.hypot(length, fp.half_width))

    def danger(self, ranges: np.ndarray) -> np.ndarray:
        """Return 0 (free beyond safe_distance) .. 1 (at the robot) for every sector."""
        safe = self.shared.safe_distance
        return np.clip(1.0 - ranges / safe, 0.0, 1.0)

    @abstractmethod
    def compute(self, angles: np.ndarray, ranges: np.ndarray, scan: SectorScanData):
        """Return the raw AvoidanceOut from sector angles and cleaned ranges (blocked: 0)."""


POLICIES: Registry[AvoidancePolicy] = Registry('x30.avoidance', AvoidancePolicy)
