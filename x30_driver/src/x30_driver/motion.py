"""Velocity policy of the X30 driver: what goes to the motion host, and when. No ROS."""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from x30_driver import protocol as p

EPSILON = 1e-3
ZERO = (0.0, 0.0, 0.0)


@dataclass
class VelocityConfig:
    """Limits and timing of the velocity stream."""

    max_vx: float = 1.0
    max_vy: float = 0.5
    max_wz: float = 1.0
    cmd_timeout_sec: float = 0.5
    stop_burst_sec: float = 0.5
    invert_vy: bool = False
    invert_wz: bool = False
    dead_zone: int = p.AXIS_DEAD_ZONE


def clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]; a non-finite value becomes 0."""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def is_motion(cmd: Tuple[float, float, float]) -> bool:
    """Return True when any component asks for motion."""
    return any(abs(v) > EPSILON for v in cmd)


class VelocityPolicy:
    """Streams cmd_vel while it is fresh, then zeros for a moment, then nothing."""

    def __init__(self, config: VelocityConfig):
        """Start idle: nothing is sent before the first command."""
        self.config = config
        self._cmd = ZERO
        self._stamp = None  # type: Optional[float]
        self._last_motion = None  # type: Optional[float]

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a new cmd_vel (base_link, REP-103), clamped to the configured limits."""
        c = self.config
        self._cmd = (clamp(vx, abs(c.max_vx)), clamp(vy, abs(c.max_vy)), clamp(wz, abs(c.max_wz)))
        self._stamp = now
        if is_motion(self._cmd):
            self._last_motion = now

    def stop(self, now: float):
        """Drop the current command; zeros are streamed until the stream runs out."""
        self._cmd = ZERO
        self._stamp = now

    @property
    def current(self) -> Tuple[float, float, float]:
        """Return the last command after clamping."""
        return self._cmd

    def fresh(self, now: float) -> bool:
        """Return True while the last command is younger than the timeout."""
        return self._stamp is not None and now - self._stamp <= self.config.cmd_timeout_sec

    def moving(self, now: float) -> bool:
        """Return True while a fresh command asks for motion."""
        return self.fresh(now) and is_motion(self._cmd)

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero command arrived within the window."""
        return self._last_motion is not None and now - self._last_motion <= window

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return what the stream sends now: the command, or zeros."""
        return self._cmd if self.fresh(now) else ZERO

    def tick(self, now: float) -> Optional[Tuple[float, float, float]]:
        """Return the velocity to send this period, or None for silence."""
        c = self.config
        if self._stamp is None:
            return None
        age = now - self._stamp
        if age <= c.cmd_timeout_sec:
            return self._cmd
        if age <= c.cmd_timeout_sec + c.stop_burst_sec:
            return ZERO
        return None

    def axes(self, cmd: Tuple[float, float, float], gait_state: Optional[int],
             crawl: bool = False) -> Tuple[int, int, int]:
        """Return (forward, lateral, turn) axis values for a command, in the current gait."""
        c = self.config
        vx, vy, wz = cmd
        vy = -vy if c.invert_vy else vy
        wz = -wz if c.invert_wz else wz
        return p.velocity_to_axes(vx, vy, wz, p.gait_limits(gait_state, crawl), c.dead_zone)


@dataclass
class AutoConfig:
    """Mode and stepping switches made on the way into and out of motion."""

    auto_non_manual_mode: bool = True
    auto_stepping: bool = True
    mode_refresh_sec: float = 2.0
    stepping_retry_sec: float = 2.0
    idle_stop_stepping_sec: float = 0.0  # 0: never


class AutoSwitch:
    """Decides the control mode and stepping commands around motion."""

    def __init__(self, config: AutoConfig):
        """Nothing sent yet."""
        self.config = config
        self._mode_sent = None  # type: Optional[float]
        self._stepping_sent = None  # type: Optional[float]
        self._last_motion = None  # type: Optional[float]

    @staticmethod
    def _due(sent: Optional[float], now: float, period: float) -> bool:
        return sent is None or now - sent >= period

    def decide(self, now: float, moving: bool, basic_state: Optional[int],
               non_manual: Optional[bool]) -> List[int]:
        """
        Return the command bases to send now.

        moving: a fresh non-zero cmd_vel. basic_state / non_manual: from the reports, None when
        unknown. Non-manual mode is asked for while the robot is not known to be in it; the
        stepping toggle only goes out from torque-controlled standing (it is a toggle).
        """
        c = self.config
        out = []
        if moving:
            self._last_motion = now
            if c.auto_non_manual_mode and non_manual is not True and self._due(
                    self._mode_sent, now, c.mode_refresh_sec):
                out.append(p.NON_MANUAL_MODE)
                self._mode_sent = now
            if c.auto_stepping and basic_state == p.STATE_TORQUE_STANDING and self._due(
                    self._stepping_sent, now, c.stepping_retry_sec):
                out.append(p.STEPPING)
                self._stepping_sent = now
            return out
        if (c.idle_stop_stepping_sec > 0.0 and basic_state == p.STATE_STEPPING
                and self._last_motion is not None
                and now - self._last_motion >= c.idle_stop_stepping_sec
                and self._due(self._stepping_sent, now, c.stepping_retry_sec)):
            out.append(p.STEPPING)
            self._stepping_sent = now
            self._last_motion = None  # once per motion: stepping started by hand stays
        return out
