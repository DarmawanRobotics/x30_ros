"""Planar velocity commands as the core sees them (the node converts Twist at the edge)."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Command:
    """A planar velocity in the robot frame: vx, vy (m/s) and wz (rad/s)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def speed(self) -> float:
        """Linear speed (m/s)."""
        return math.hypot(self.vx, self.vy)

    def is_finite(self) -> bool:
        """Return True when every axis is a finite number."""
        return math.isfinite(self.vx) and math.isfinite(self.vy) and math.isfinite(self.wz)

    def is_neutral(self, eps: float) -> bool:
        """Return True when every axis is below eps in magnitude (a released stick)."""
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.wz) < eps

    def scaled(self, linear: float, angular: float | None = None) -> Command:
        """Return the command with vx, vy scaled by linear and wz by angular (default: linear)."""
        angular = linear if angular is None else angular
        return Command(self.vx * linear, self.vy * linear, self.wz * angular)


ZERO = Command()


def speed_ratio(before: Command, after: Command, eps: float = 1e-9) -> float:
    """Return the smallest factor by which a commanded axis was reduced (1: untouched)."""
    ratios = []
    if before.speed > eps:
        ratios.append(after.speed / before.speed)
    if abs(before.wz) > eps:
        ratios.append(abs(after.wz) / abs(before.wz))
    return max(0.0, min(1.0, *ratios)) if ratios else 1.0
