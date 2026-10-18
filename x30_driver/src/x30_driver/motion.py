"""
Velocity stream of the X30 driver: what goes to the motion host, and when. No ROS.

The rules (shared by the Deep Robotics drivers):

- An episode starts with the first motion command after cmd_vel was quiet for
  cmd_timeout_sec + stop_burst_sec (or the first ever). Only then, and only with the gate open,
  the driver switches the robot to the control mode that obeys it and starts stepping from
  torque-controlled standing: a few tries each, never again in that episode.
- The gate closes on a hold (sit_down, emergency_stop, stop_stepping; released by stand_up or
  start_stepping), on a takeover (the robot leaves that control mode after it was seen in it;
  the latch ends only when the robot reports that mode again) and while the robot reports an
  emergency stop or fall. After it opens again, motion goes out only once cmd_vel has been zero
  (one message) or quiet (cmd_timeout_sec): a stream that kept running never drives the robot
  off. Nothing re-arms on a timer.
- Zeros go out for stop_burst_sec after the last non-zero command or after the gate closed, then
  nothing, even while zero Twists keep arriving.
"""

import math
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from x30_driver import protocol as p

EPSILON = p.VELOCITY_EPSILON
ZERO = (0.0, 0.0, 0.0)
ZERO_AXES = (0, 0, 0)
MODE_TRIES = 3          # control mode requests at the start of an episode
MODE_TRY_SEC = 0.5      # between them
STEPPING_TRIES = 3      # stepping toggles at the start of an episode (spaced by the ToggleGuard)


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


def mode_name(non_manual: bool) -> str:
    """Return the readable name of a control mode."""
    return 'non-manual' if non_manual else 'manual'


class VelocityPolicy:
    """The last cmd_vel: clamped, how old, and the axis values it means in a gait."""

    def __init__(self, config: VelocityConfig):
        """No command yet."""
        self.config = config
        self._cmd = ZERO
        self._stamp = None  # type: Optional[float]
        self._last_motion = None  # type: Optional[float]

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a cmd_vel (base_link, REP-103); a non-finite component makes it a stop."""
        c = self.config
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            vx = vy = wz = 0.0
        self._cmd = (clamp(vx, abs(c.max_vx)), clamp(vy, abs(c.max_vy)), clamp(wz, abs(c.max_wz)))
        self._stamp = now
        if is_motion(self._cmd):
            self._last_motion = now

    @property
    def current(self) -> Tuple[float, float, float]:
        """Return the last command after clamping."""
        return self._cmd

    @property
    def stamp(self) -> Optional[float]:
        """Return when the last command arrived, None before the first."""
        return self._stamp

    @property
    def last_motion(self) -> Optional[float]:
        """Return when the last non-zero command arrived."""
        return self._last_motion

    def fresh(self, now: float) -> bool:
        """Return True while the last command is younger than the timeout."""
        return self._stamp is not None and now - self._stamp <= self.config.cmd_timeout_sec

    def moving(self, now: float) -> bool:
        """Return True while a fresh command asks for motion."""
        return self.fresh(now) and is_motion(self._cmd)

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero command arrived within the window."""
        return self._last_motion is not None and now - self._last_motion <= window

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
    """Mode and stepping switches made at the start of an episode."""

    auto_control_mode: bool = True
    auto_stepping: bool = True
    idle_stop_stepping_sec: float = 0.0  # 0: never
    # the mode in which the robot obeys this driver: non-manual for the autonomous prefix 0x31,
    # manual for 0x21 (the driver then acts as the handheld)
    target_non_manual: bool = True


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
    """cmd_vel watchdog, episode switches, gate and zero burst: one step per period."""

    def __init__(self, velocity: VelocityConfig, auto: AutoConfig, mode: str = 'axis',
                 toggle_timeout_sec: float = 5.0):
        """mode: 'axis' (axis commands) or 'topic' (republish the velocity)."""
        self.policy = VelocityPolicy(velocity)
        self.config = velocity
        self.auto = auto
        self.mode = mode
        self.toggles = ToggleGuard(toggle_timeout_sec)
        self._lock = threading.Lock()
        self._hold = None  # type: Optional[str]
        self._latch = None  # type: Optional[str]
        self._end_on_target = False
        self._target_seen = False
        self._disarm = None  # type: Optional[str]
        self._armed = True
        self._episode = False
        self._episode_new = False
        self._mode_phase = False
        self._mode_tries = 0
        self._mode_try_at = None  # type: Optional[float]
        self._step_phase = False
        self._step_tries = 0
        self._drove = False
        self._burst_until = None  # type: Optional[float]

    # -------------------------------------------------------------- inputs

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a cmd_vel message (a non-finite component makes it a stop)."""
        with self._lock:
            previous = self.policy.stamp
            self.policy.command(vx, vy, wz, now)
            if not is_motion(self.policy.current):
                if self._open():
                    self._armed = True  # a zero after the release: what follows is no stale stream
                return
            c = self.config
            if previous is None or now - previous > c.cmd_timeout_sec + c.stop_burst_sec:
                self._end_episode()
            if self._open() and self._armed and not self._episode:
                self._episode = True
                self._episode_new = True

    def hold(self, now: float, service: str):
        """Hold until release() (stand_up, start_stepping) and then a zero or quiet cmd_vel."""
        with self._lock:
            self._hold = service
            self._close(now)

    def release(self) -> bool:
        """End a hold; return True when there was one."""
        with self._lock:
            held, self._hold = self._hold is not None, None
            return held

    def latch(self, now: float, reason: str):
        """Take the robot out of the mode that obeys the driver: a takeover latch."""
        with self._lock:
            if self._latch is None:
                self._latch = reason
                self._end_on_target = False
                self._close(now)

    def request_handback(self):
        """Note that the target mode was asked for: the latch ends once the robot reports it."""
        with self._lock:
            if self._latch is not None:
                self._end_on_target = True

    # -------------------------------------------------------------- state

    def _open(self) -> bool:
        return self._hold is None and self._latch is None and self._disarm is None

    def _close(self, now: float):
        self._armed = False
        self._end_episode()
        self._burst_until = now + self.config.stop_burst_sec

    def _end_episode(self):
        self._episode = self._episode_new = False
        self._mode_phase = self._step_phase = False

    def _gate(self, now: float) -> str:
        if self._hold is not None:
            return 'hold:' + self._hold
        if self._latch is not None:
            return 'takeover ({} mode)'.format(mode_name(not self.auto.target_non_manual))
        if self._disarm is not None:
            return 'disarmed:' + self._disarm
        if not self._armed:
            return 'waiting for zero or quiet cmd_vel'
        return 'driving' if self.policy.moving(now) else 'idle'

    def gate(self, now: float) -> str:
        """
        Return the gate as the state shows it.

        driving, idle, hold:<service>, takeover (<mode> mode), disarmed:<reason> or waiting for
        zero or quiet cmd_vel.
        """
        with self._lock:
            return self._gate(now)

    def held(self, now: float) -> bool:
        """Return True while cmd_vel asks for motion and the gate holds it back."""
        with self._lock:
            return self.policy.moving(now) and not (self._open() and self._armed)

    @property
    def latch_reason(self) -> Optional[str]:
        """Return why the takeover latch is on, None when it is off."""
        return self._latch

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero cmd_vel arrived within the window."""
        with self._lock:
            return self.policy.recently_moved(now, window)

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return the velocity that goes out: the command while driving, zeros otherwise."""
        with self._lock:
            if self.policy.moving(now) and self._open() and self._armed:
                return self.policy.current
            return ZERO

    # -------------------------------------------------------------- one period

    def step(self, now: float, view: RobotView) -> StreamStep:
        """Return the commands, axis values or twist of this period."""
        with self._lock:
            c = self.config
            target = self.auto.target_non_manual
            in_target = None if view.non_manual is None else bool(view.non_manual) == target
            if view.basic_state == p.STATE_EMERGENCY:
                if self._disarm is None:
                    self._disarm = 'emergency stop or fall'
                    self._close(now)
            elif view.basic_state is not None:
                self._disarm = None  # the report left the emergency state
            if in_target:
                if self._latch is not None and self._end_on_target:
                    self._latch = None  # handed back: the robot reports the target mode again
                self._target_seen = True
            elif in_target is False:
                if self._latch is not None:
                    self._end_on_target = True
                elif self._target_seen:
                    self._latch = 'operator took over ({} mode)'.format(mode_name(not target))
                    self._end_on_target = True
                    self._close(now)
                self._target_seen = False
            stamp = self.policy.stamp
            if self._open() and not self._armed and not self.policy.fresh(now):
                self._armed = True  # cmd_vel went quiet after the release
            if self._episode and stamp is not None and (
                    now - stamp > c.cmd_timeout_sec + c.stop_burst_sec):
                self._end_episode()
            moving = self.policy.moving(now)
            step = StreamStep(self._switches(now, view, in_target, moving))
            self._emit(now, view, in_target, moving, step)
            return step

    def _switches(self, now: float, view: RobotView, in_target: Optional[bool],
                  moving: bool) -> List[int]:
        """Return the mode and stepping commands: at the start of an episode only."""
        a = self.auto
        out = []
        if not (self._open() and self._armed):
            return out
        if self._episode:
            if self._episode_new:
                self._episode_new = False
                # decided once: target mode only on a fresh report that the robot is not in it
                self._mode_phase = a.auto_control_mode and in_target is False
                self._mode_tries, self._mode_try_at = 0, None
                self._step_phase = (a.auto_stepping
                                    and view.basic_state == p.STATE_TORQUE_STANDING)
                self._step_tries = 0
            if self._mode_phase:
                if in_target:
                    self._mode_phase = False
                elif in_target is False and (self._mode_try_at is None
                                             or now - self._mode_try_at >= MODE_TRY_SEC):
                    if self._mode_tries < MODE_TRIES:
                        out.append(p.NON_MANUAL_MODE if a.target_non_manual else p.MANUAL_MODE)
                        self._mode_tries += 1
                        self._mode_try_at = now
                    else:
                        self._mode_phase = False  # gave up
            if self._step_phase:
                state = view.basic_state
                if state == p.STATE_TORQUE_STANDING:
                    if self._step_tries < STEPPING_TRIES:
                        if self.toggles.request(now, p.STEPPING, state):
                            out.append(p.STEPPING)
                            self._step_tries += 1
                    elif self.toggles.pending(now, state) is None:
                        self._step_phase = False  # gave up
                elif state is not None:
                    self._step_phase = False  # stepping, or somewhere stepping cannot start
        if (a.idle_stop_stepping_sec > 0.0 and self._drove and not moving
                and view.basic_state == p.STATE_STEPPING
                and self.policy.last_motion is not None
                and now - self.policy.last_motion >= a.idle_stop_stepping_sec
                and self.toggles.request(now, p.STEPPING, view.basic_state)):
            out.append(p.STEPPING)
            self._drove = False  # once per drive: stepping started by hand stays
        return out

    def _emit(self, now: float, view: RobotView, in_target: Optional[bool], moving: bool,
              step: StreamStep):
        """Fill in the axis values or the twist, and why cmd_vel does not move the robot."""
        c = self.config
        driving = moving and self._open() and self._armed
        burst = self._burst_until is not None and now <= self._burst_until
        if moving and not driving:
            step.ignored = self._gate(now)
        want = self.policy.current if driving else None
        if self.mode == 'topic':
            if want is not None:
                step.twist = want
                self._burst_until = now + c.stop_burst_sec
            elif burst:
                step.twist = ZERO
            return
        if view.basic_state != p.STATE_STEPPING:
            if want is not None:
                step.ignored = ('no fresh motion state report' if view.basic_state is None
                                else 'not stepping ({})'.format(
                                    p.name_of(p.BASIC_STATES, view.basic_state)))
            return
        axes = None if want is None else self.policy.axes(want, view.gait_state, view.crawl)
        if want is not None and axes is None:
            step.ignored = 'no speed limits for the {} gait (gait_limits)'.format(
                p.name_of(p.GAIT_STATES, view.gait_state))
        if axes is not None and axes != ZERO_AXES:
            step.axes = axes
            self._burst_until = now + c.stop_burst_sec
            self._drove = True
            if in_target is False:
                step.ignored = '{} mode: the robot obeys the {} side'.format(
                    mode_name(view.non_manual), 'perception' if view.non_manual else 'handheld')
        elif burst:
            step.axes = ZERO_AXES
