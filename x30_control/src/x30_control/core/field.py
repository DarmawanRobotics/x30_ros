"""Protective field: limit a command to what the robot can still stop within."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from x30_control.core.command import ZERO, Command, speed_ratio
from x30_control.core.config import FieldConfig
from x30_control.core.scan import ScanState, ScanView
from x30_control.core.separation import separation_speed
from x30_control.core.sweep import swept_clearance
from x30_utils.robot import RobotProfile
from x30_utils.safety import ArcCheck, rotation_room, stopping_yaw_rate

EPS = 1e-6
# a command on the field's boundary passes untouched: what it is over by there is floating-point
# noise (the follower's guard checks with the field's margins, and lands right on it)
TOLERANCE = 1e-9


class FieldState(Enum):
    """What the field did to the command (ControlStatus.FIELD_*)."""

    OFF = 'off'
    CLEAR = 'clear'
    LIMITED = 'limited'
    STOP = 'stop'
    STALE = 'stale'
    BLIND = 'blind'


@dataclass(frozen=True)
class FieldResult:
    """The limited command, the field state, clearance (m, inf: clear) and the applied scale."""

    command: Command
    state: FieldState
    clearance: float = math.inf
    scale: float = 1.0


class ProtectiveField:
    """Speed and separation monitoring on the 360 deg safety scan."""

    def __init__(self, robot: RobotProfile, config: FieldConfig):
        self.footprint = robot.footprint
        self.limits = robot.limits
        self.config = config
        self.arc = ArcCheck(robot.footprint, robot.limits, config.lateral_margin)

    def limit(self, cmd: Command, view: ScanView, remote: bool) -> FieldResult:
        """Return the command limited for the scan view; remote selects REMOTE behaviour."""
        if view.state is not ScanState.OK:
            return self._degraded(cmd, view, remote)
        clearance = self.clearance(cmd, view.points)
        if remote and not self.config.in_remote:
            return FieldResult(cmd, FieldState.OFF, clearance)
        margin = self.config.remote_margin if remote else self.config.margin
        out = self.apply(cmd, view.points, margin)
        return FieldResult(out, _state(cmd, out), clearance, speed_ratio(cmd, out))

    def clearance(self, cmd: Command, points: np.ndarray) -> float:
        """Free distance (m) of the footprint along the commanded translation (inf: none)."""
        speed = cmd.speed
        if speed <= EPS:
            return math.inf
        ux, uy = cmd.vx / speed, cmd.vy / speed
        return swept_clearance(self.footprint, ux, uy, points, self.config.lateral_margin)

    def apply(self, cmd: Command, points: np.ndarray, margin: float) -> Command:
        """Limit translation, rotation and their combination for obstacle points (robot frame)."""
        vx, vy = self._translation(cmd, points, margin)
        out = Command(vx, vy, self._rotation(cmd.wz, points))
        lim = self.limits
        person = self.config.human_speed * (lim.reaction_time + self.arc.braking_time(out))
        scale = self.arc.scale(out, points, margin + person, self.config.yaw_margin)
        if scale >= 1.0 - TOLERANCE:
            return out
        out = out.scaled(scale)
        vx, vy, wz = out.vx, out.vy, out.wz
        if out.speed < self.config.min_speed:
            vx = vy = 0.0
        if abs(wz) < self.config.min_yaw_rate:
            wz = 0.0
        return Command(vx, vy, wz)

    def capped(self, cmd: Command, speed: float) -> Command:
        """Scale the command so no point of the footprint moves faster than speed (m/s)."""
        fastest = cmd.speed + abs(cmd.wz) * self.footprint.circumscribed_radius
        if fastest <= speed:
            return cmd
        return cmd.scaled(speed / fastest)

    def _translation(self, cmd: Command, points: np.ndarray, margin: float) -> tuple:
        speed = cmd.speed
        if speed <= EPS:
            return cmd.vx, cmd.vy
        room = self.clearance(cmd, points) - margin
        lim = self.limits
        allowed = separation_speed(room, lim.max_decel, lim.reaction_time, self.config.human_speed)
        if allowed >= speed - TOLERANCE:
            return cmd.vx, cmd.vy
        if allowed < self.config.min_speed:
            return 0.0, 0.0
        scale = allowed / speed
        return cmd.vx * scale, cmd.vy * scale

    def _rotation(self, wz: float, points: np.ndarray) -> float:
        """Limit the yaw rate by the free turning angle of the footprint (finding C1)."""
        if abs(wz) <= EPS:
            return wz
        room = rotation_room(self.footprint, points, wz) - self.config.yaw_margin
        allowed = stopping_yaw_rate(room, self.limits.max_yaw_accel, self.limits.reaction_time)
        if allowed >= abs(wz) - TOLERANCE:
            return wz
        if allowed < self.config.min_yaw_rate:
            return 0.0
        return math.copysign(allowed, wz)

    def _degraded(self, cmd: Command, view: ScanView, remote: bool) -> FieldResult:
        """STALE or BLIND: AUTO stops; REMOTE creeps (and keeps the field on what is seen)."""
        state = FieldState.STALE if view.state is ScanState.STALE else FieldState.BLIND
        if not remote:
            return FieldResult(ZERO, state, 0.0, 0.0)
        out = self.capped(cmd, self.config.remote_degraded_speed)
        if state is FieldState.BLIND and self.config.in_remote:
            out = self.apply(out, view.points, self.config.remote_margin)
            if _toward_blocked(out, view):
                out = Command(0.0, 0.0, out.wz)
        return FieldResult(out, state, 0.0, speed_ratio(cmd, out))


def _toward_blocked(cmd: Command, view: ScanView) -> bool:
    """Return True when the translation heads toward a blocked sector."""
    angles = view.blocked_angles
    total = round(2.0 * math.pi / view.sector_width) if view.sector_width > 0.0 else 0
    if cmd.speed <= EPS or angles.size == 0 or angles.size >= total:
        return False
    heading = math.atan2(cmd.vy, cmd.vx)
    return bool(np.any(np.cos(angles - heading) > -math.sin(0.5 * view.sector_width)))


def _state(cmd: Command, out: Command) -> FieldState:
    if out == cmd:
        return FieldState.CLEAR
    if out == ZERO:
        return FieldState.STOP
    return FieldState.LIMITED
