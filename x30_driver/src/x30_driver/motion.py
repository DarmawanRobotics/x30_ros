"""Velocity policy of the X30 driver: what goes to the motion host, and when. No ROS."""

import math
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from x30_driver import protocol as p

EPSILON = p.VELOCITY_EPSILON
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
    # gait speeds the axes are scaled with: Appendix B plus the gait_limits parameter
    gait_limits: Dict[int, p.GaitLimits] = field(default_factory=p.limits_table)


def clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]; a non-finite value becomes 0."""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def is_motion(cmd: Tuple[float, float, float]) -> bool:
    """Return True when any component asks for motion."""
    return any(abs(v) >= EPSILON for v in cmd)


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

    def limits(self, gait_state: Optional[int], crawl: bool = False) -> Optional[p.GaitLimits]:
        """Return a gait's speed limits, None when neither Appendix B nor gait_limits has it."""
        return p.gait_limits(gait_state, crawl, self.config.gait_limits)

    def axes(self, cmd: Tuple[float, float, float], gait_state: Optional[int],
             crawl: bool = False) -> Optional[Tuple[int, int, int]]:
        """Return (forward, lateral, turn) axis values in the current gait, None without limits."""
        limits = self.limits(gait_state, crawl)
        if limits is None:
            return None
        c = self.config
        vx, vy, wz = cmd
        vy = -vy if c.invert_vy else vy
        wz = -wz if c.invert_wz else wz
        return p.velocity_to_axes(vx, vy, wz, limits, c.dead_zone)


@dataclass
class AutoConfig:
    """Mode and stepping switches made on the way into and out of motion."""

    auto_control_mode: bool = True
    auto_stepping: bool = True
    mode_refresh_sec: float = 2.0
    stepping_retry_sec: float = 2.0
    idle_stop_stepping_sec: float = 0.0  # 0: never
    # the mode in which the robot obeys this driver: non-manual for the autonomous prefix 0x31,
    # manual for 0x21 (the driver then acts as the handheld)
    target_non_manual: bool = True
    rearm_quiet_sec: float = 0.5         # disarmed until cmd_vel was quiet or zero this long


def mode_name(non_manual: bool) -> str:
    """Return the readable name of a control mode."""
    return 'non-manual' if non_manual else 'manual'


class AutoSwitch:
    """
    Decides the control mode and stepping commands around motion, and when to hold back.

    Disarmed, it sends nothing and the velocity stream sends only zeros: after an operator
    takeover (the robot leaves the target mode while cmd_vel moves it), after a stop requested
    through a service (disarm()), and while the robot is in emergency stop or fall. It arms
    again once cmd_vel has been quiet or zero for rearm_quiet_sec, or by arm().
    """

    def __init__(self, config: AutoConfig):
        """Armed, nothing sent yet."""
        self.config = config
        self.armed = True
        self.disarm_reason = ''
        self._disarmed_at = None  # type: Optional[float]
        self._target_seen = False
        self._mode_sent = None  # type: Optional[float]
        self._stepping_sent = None  # type: Optional[float]
        self._last_motion = None  # type: Optional[float]
        self._last_stepping_motion = None  # type: Optional[float]

    @property
    def mode_command(self) -> int:
        """Return the command base that puts the robot in the target mode."""
        return p.NON_MANUAL_MODE if self.config.target_non_manual else p.MANUAL_MODE

    def disarm(self, now: float, reason: str):
        """Hold back until cmd_vel has been quiet for rearm_quiet_sec (or arm())."""
        self.armed = False
        self.disarm_reason = reason
        self._disarmed_at = now

    def arm(self):
        """Arm at once (an explicit request for the target mode)."""
        self.armed = True
        self.disarm_reason = ''

    @staticmethod
    def _due(sent: Optional[float], now: float, period: float) -> bool:
        return sent is None or now - sent >= period

    def decide(self, now: float, moving: bool, basic_state: Optional[int],
               non_manual: Optional[bool]) -> List[int]:
        """
        Return the command bases to send now.

        moving: a fresh non-zero cmd_vel. basic_state / non_manual: from fresh reports, None
        when unknown. The target mode is asked for while the robot is not known to be in it;
        the stepping toggle only goes out from torque-controlled standing (it is a toggle).
        """
        c = self.config
        if moving:
            self._last_motion = now
        if basic_state == p.STATE_EMERGENCY:
            self.disarm(now, 'emergency stop or fall')
        in_target = None if non_manual is None else bool(non_manual) == c.target_non_manual
        if in_target:
            self._target_seen = True
        elif in_target is False and self._target_seen:
            self._target_seen = False
            if moving and self.armed:
                self.disarm(now, 'operator took over ({} mode)'.format(
                    mode_name(not c.target_non_manual)))
        if not self.armed:
            quiet_since = max(self._disarmed_at, self._last_motion or self._disarmed_at)
            if moving or basic_state == p.STATE_EMERGENCY or now - quiet_since < c.rearm_quiet_sec:
                return []
            self.arm()
        out = []
        if moving:
            self._last_stepping_motion = now
            if c.auto_control_mode and in_target is not True and self._due(
                    self._mode_sent, now, c.mode_refresh_sec):
                out.append(self.mode_command)
                self._mode_sent = now
            if c.auto_stepping and basic_state == p.STATE_TORQUE_STANDING and self._due(
                    self._stepping_sent, now, c.stepping_retry_sec):
                out.append(p.STEPPING)
                self._stepping_sent = now
            return out
        if (c.idle_stop_stepping_sec > 0.0 and basic_state == p.STATE_STEPPING
                and self._last_stepping_motion is not None
                and now - self._last_stepping_motion >= c.idle_stop_stepping_sec
                and self._due(self._stepping_sent, now, c.stepping_retry_sec)):
            out.append(p.STEPPING)
            self._stepping_sent = now
            self._last_stepping_motion = None  # once per motion: stepping started by hand stays
        return out


class ToggleGuard:
    """
    One stand/sit or stepping toggle at a time.

    The two commands alternate states (1.2.2), so a second one sent before the first shows in
    the reported basic state would undo it. A toggle counts as done once basic_state differs
    from the state it was sent in, or after timeout_sec.
    """

    def __init__(self, timeout_sec: float):
        """No toggle pending."""
        self.timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._pending = None  # type: Optional[Tuple[int, float, Optional[int]]]

    def _check(self, now: float, basic_state: Optional[int]) -> Optional[int]:
        if self._pending is None:
            return None
        command, sent_at, from_state = self._pending
        if (basic_state is not None and basic_state != from_state) or (
                now - sent_at >= self.timeout_sec):
            self._pending = None
            return None
        return command

    def pending(self, now: float, basic_state: Optional[int]) -> Optional[int]:
        """Return the toggle not yet seen in basic_state, None when there is none."""
        with self._lock:
            return self._check(now, basic_state)

    def request(self, now: float, command: int, basic_state: Optional[int]) -> bool:
        """Return True and record the toggle when it may go out now."""
        with self._lock:
            if self._check(now, basic_state) is not None:
                return False
            self._pending = (command, now, basic_state)
            return True

    def cancel(self):
        """Forget the pending toggle (it could not be sent)."""
        with self._lock:
            self._pending = None


@dataclass
class RobotView:
    """What fresh reports say, as the velocity stream needs it (None: unknown or stale)."""

    basic_state: Optional[int] = None    # from a fresh MotionStateData (0x1009) only
    gait_state: Optional[int] = None
    non_manual: Optional[bool] = None    # from a fresh RcsData (0x1008)
    crawl: bool = False                  # body height report (0x11050F08) says crawl


@dataclass
class StreamStep:
    """What goes out in one period of the velocity stream."""

    commands: List[int] = field(default_factory=list)   # mode and stepping command bases
    axes: Optional[Tuple[int, int, int]] = None          # forward, lateral, turn values
    twist: Optional[Tuple[float, float, float]] = None   # velocity_mode topic
    ignored: str = ''                                    # why cmd_vel does not move the robot


class VelocityStream:
    """cmd_vel watchdog, automatic switches and gating together: one step per period."""

    def __init__(self, velocity: VelocityConfig, auto: AutoConfig, mode: str = 'axis'):
        """mode: 'axis' (axis commands) or 'topic' (republish the velocity)."""
        self.policy = VelocityPolicy(velocity)
        self.auto = AutoSwitch(auto)
        self.mode = mode
        self._lock = threading.Lock()

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a cmd_vel message."""
        with self._lock:
            self.policy.command(vx, vy, wz, now)

    def stop(self, now: float):
        """Drop the current command (zeros, then silence)."""
        with self._lock:
            self.policy.stop(now)

    def disarm(self, now: float, reason: str):
        """Hold back (see AutoSwitch) and drop the current command."""
        with self._lock:
            self.auto.disarm(now, reason)
            self.policy.stop(now)

    def arm(self):
        """Arm at once."""
        with self._lock:
            self.auto.arm()

    @property
    def armed(self) -> bool:
        """Return True while motion commands may go out."""
        return self.auto.armed

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero cmd_vel arrived within the window."""
        with self._lock:
            return self.policy.recently_moved(now, window)

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return the velocity the stream is sending (zeros while disarmed)."""
        with self._lock:
            return self.policy.sent_command(now) if self.auto.armed else ZERO

    def step(self, now: float, view: RobotView) -> StreamStep:
        """Return the commands, axis values or twist of this period."""
        with self._lock:
            moving = self.policy.moving(now)
            cmd = self.policy.tick(now)
            step = StreamStep(self.auto.decide(now, moving, view.basic_state, view.non_manual))
            if cmd is None:
                return step
            if not self.auto.armed:
                cmd = ZERO
                if moving:
                    step.ignored = 'disarmed: {} (until cmd_vel is quiet for {:.1f} s)'.format(
                        self.auto.disarm_reason, self.auto.config.rearm_quiet_sec)
            if self.mode == 'topic':
                step.twist = cmd
                return step
            if view.basic_state != p.STATE_STEPPING:
                if moving and not step.ignored:
                    step.ignored = ('no fresh motion state report' if view.basic_state is None
                                    else 'not stepping ({})'.format(
                                        p.name_of(p.BASIC_STATES, view.basic_state)))
                return step
            axes = self.policy.axes(cmd, view.gait_state, view.crawl)
            if axes is None:
                step.axes = (0, 0, 0)
                if moving and not step.ignored:
                    step.ignored = 'no speed limits for the {} gait (gait_limits)'.format(
                        p.name_of(p.GAIT_STATES, view.gait_state))
                return step
            step.axes = axes
            target = self.auto.config.target_non_manual
            if moving and not step.ignored and view.non_manual is not None and (
                    bool(view.non_manual) != target):
                step.ignored = '{} mode: the robot obeys the {} side'.format(
                    mode_name(view.non_manual), 'perception' if view.non_manual else 'handheld')
            return step
