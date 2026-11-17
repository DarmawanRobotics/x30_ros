"""The last checks before the robot: speed envelope, health speed scale, protective field."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_control.core.command import ZERO, Command, speed_ratio
from x30_control.core.config import FieldConfig, MuxConfig
from x30_control.core.envelope import Envelope, HealthLimit
from x30_control.core.field import FieldResult, FieldState, ProtectiveField
from x30_control.core.scan import ScanData, ScanMonitor, ScanView
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class GateResult:
    """The command for the robot and what limited it."""

    command: Command
    field: FieldState
    clearance: float = math.inf
    speed_scale: float = 1.0
    finite: bool = True
    clipped: bool = False
    health_scale: float = 1.0


class Gate:
    """Envelope (clamp first), then the health scale, then the protective field."""

    def __init__(self, robot: RobotProfile, mux: MuxConfig, field: FieldConfig):
        self.envelope = Envelope(robot)
        self.health = HealthLimit(mux.use_health_limit, mux.health_timeout_sec)
        self.scan = ScanMonitor(robot.frames.base, field)
        self.field = ProtectiveField(robot, field)

    def update_scan(self, scan: ScanData, received: float) -> str:
        """Store a safety scan (monotonic receive time); return its problem or ''."""
        return self.scan.update(scan, received)

    def view(self, received_now: float, now: float) -> ScanView:
        """Return the current scan view (monotonic and node-clock times)."""
        return self.scan.view(received_now, now)

    def apply(self, cmd: Command, remote: bool, received_now: float, now: float) -> GateResult:
        """Return the command that may reach the robot (monotonic and node-clock times)."""
        env = self.envelope.apply(cmd)
        if not env.finite:
            return GateResult(ZERO, FieldState.STOP, 0.0, 0.0, finite=False)
        health = self.health.scale(received_now)
        limited = env.command.scaled(health) if health < 1.0 else env.command
        result: FieldResult = self.field.limit(limited, self.view(received_now, now), remote)
        scale = speed_ratio(cmd, result.command)
        return GateResult(
            result.command, result.state, result.clearance, scale, True, env.clipped, health
        )

    def idle_state(self, remote: bool, received_now: float, now: float) -> FieldResult:
        """Return the field state for a robot standing still (status while nothing drives)."""
        return self.field.limit(ZERO, self.view(received_now, now), remote)
