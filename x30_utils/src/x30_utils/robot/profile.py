"""The robot description every package reads: footprint, height, limits, kinematics, frames."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.config import nested, param

KINEMATICS = ('omni', 'diff')


@dataclass(frozen=True)
class Footprint:
    """Rectangular footprint around base_link (x forward, y left)."""

    front: float = param(0.40, min=0.01, max=5.0, unit='m', doc='base_link to the front edge')
    rear: float = param(0.40, min=0.01, max=5.0, unit='m', doc='base_link to the rear edge')
    half_width: float = param(0.28, min=0.01, max=5.0, unit='m', doc='half of the body width')

    @property
    def inscribed_radius(self) -> float:
        """Largest circle around base_link inside the footprint."""
        return min(self.front, self.rear, self.half_width)

    @property
    def circumscribed_radius(self) -> float:
        """Smallest circle around base_link containing the footprint (swept when turning)."""
        return math.hypot(max(self.front, self.rear), self.half_width)

    def corners(self) -> np.ndarray:
        """Return the four corners (front-left first, counter-clockwise)."""
        f, r, w = self.front, self.rear, self.half_width
        return np.array([[f, w], [-r, w], [-r, -w], [f, -w]])

    def contains(self, points: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """Return a mask of (N, 2) points inside the footprint grown by margin."""
        p = np.asarray(points, dtype=float).reshape(-1, 2)
        return (
            (p[:, 0] <= self.front + margin)
            & (p[:, 0] >= -self.rear - margin)
            & (np.abs(p[:, 1]) <= self.half_width + margin)
        )


@dataclass(frozen=True)
class Limits:
    """Motion limits and the braking model shared by every safety check."""

    max_vx: float = param(0.8, min=0.0, max=10.0, unit='m/s', doc='forward / backward speed')
    max_vy: float = param(0.4, min=0.0, max=10.0, unit='m/s', doc='sideways speed (omni only)')
    max_wz: float = param(1.0, min=0.0, max=10.0, unit='rad/s', doc='yaw rate')
    max_accel: float = param(0.5, min=0.01, max=20.0, unit='m/s^2', doc='linear acceleration')
    max_decel: float = param(0.6, min=0.05, max=20.0, unit='m/s^2', doc='braking deceleration')
    max_yaw_accel: float = param(1.5, min=0.05, max=50.0, unit='rad/s^2', doc='yaw braking')
    reaction_time: float = param(
        0.25, min=0.0, max=2.0, unit='s', doc='command to braking delay (sensing + transport)'
    )


@dataclass(frozen=True)
class Frames:
    """TF frame names (REP-105)."""

    map: str = param('map', doc='global, fixed frame')
    odom: str = param('odom', doc='continuous odometry frame')
    base: str = param('base_link', doc='robot body frame')


@dataclass(frozen=True)
class RobotProfile:
    """One description of the robot, shared by every node through the robot.* parameters."""

    name: str = param('robot', doc='robot model, for logs')
    kinematics: str = param('omni', choices=KINEMATICS, doc='omni: vx, vy, wz; diff: vx, wz')
    base_height: float = param(
        0.35, min=0.01, max=3.0, unit='m', doc='base_link above the floor when standing'
    )
    footprint: Footprint = nested(Footprint)
    limits: Limits = nested(Limits)
    frames: Frames = nested(Frames)

    @property
    def can_strafe(self) -> bool:
        """True when the robot moves sideways."""
        return self.kinematics == 'omni'

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if not self.can_strafe and self.limits.max_vy > 0.0:
            problems.append('limits.max_vy must be 0 for diff kinematics')
        if len({self.frames.map, self.frames.odom, self.frames.base}) < 3:
            problems.append('frames.map, frames.odom and frames.base must differ')
        return problems
