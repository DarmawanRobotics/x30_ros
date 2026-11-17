"""Speed envelope: robot limits and kinematics, non-finite filtering, the health speed scale."""

from __future__ import annotations

from dataclasses import dataclass

from x30_control.core.command import ZERO, Command
from x30_utils.math import clamp
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class EnvelopeResult:
    """A command inside the envelope and what had to be done to it."""

    command: Command
    finite: bool = True
    clipped: bool = False


class Envelope:
    """|vx| <= max_vx, |vy| <= max_vy (0 for diff), |wz| <= max_wz; non-finite -> zero."""

    def __init__(self, robot: RobotProfile):
        self.can_strafe = robot.can_strafe
        self.max_vx = robot.limits.max_vx
        self.max_vy = robot.limits.max_vy if robot.can_strafe else 0.0
        self.max_wz = robot.limits.max_wz

    def apply(self, cmd: Command) -> EnvelopeResult:
        """Return the command inside the envelope; x and y are scaled together (same heading)."""
        if not cmd.is_finite():
            return EnvelopeResult(ZERO, finite=False)
        vy = cmd.vy if self.can_strafe else 0.0
        scale = 1.0
        if abs(cmd.vx) > self.max_vx:
            scale = min(scale, self.max_vx / abs(cmd.vx))
        if abs(vy) > self.max_vy:
            scale = min(scale, self.max_vy / abs(vy))
        wz = max(-self.max_wz, min(self.max_wz, cmd.wz))
        out = Command(cmd.vx * scale, vy * scale, wz)
        return EnvelopeResult(out, clipped=out != cmd)


class HealthLimit:
    """Speed scale requested on health/status: 1 when disabled, not received yet or stale."""

    def __init__(self, enabled: bool, timeout_sec: float):
        self.enabled = enabled
        self.timeout_sec = timeout_sec
        self._scale = 1.0
        self._received: float | None = None

    def update(self, scale: float, now: float):
        """Store a requested scale (monotonic now); invalid values become 0, above 1 becomes 1."""
        self._scale = clamp(scale, 0.0, 1.0)
        self._received = now

    def fresh(self, now: float) -> bool:
        """Return True when a health status arrived within the timeout."""
        return self._received is not None and now - self._received <= self.timeout_sec

    def scale(self, now: float) -> float:
        """Return the factor for every axis of the command."""
        if not self.enabled or not self.fresh(now):
            return 1.0
        return self._scale
