"""Docking.tick(inputs) -> DockDecision: the docking state machine, without ROS."""

from __future__ import annotations

import math

from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.docking.control import DockController, DockLimits
from x30_navigation.core.docking.geometry import (
    DockErrors,
    DockEstimate,
    direction_sign,
    dock_errors,
    goal_from_tag,
)
from x30_navigation.core.docking.seated import SeatedPhases
from x30_navigation.core.docking.state import (
    ATTEMPT,
    BUSY,
    NO_ERRORS,
    D,
    DockDecision,
    DockInputs,
    DockReport,
    StallWatch,
    Stopwatch,
)
from x30_utils.math import Pose2D


class Docking(SeatedPhases):
    """Dock onto a charger marked by a tag, sit down, start charging; undock again."""

    def __init__(self, config: DockingConfig, can_strafe: bool, limits: DockLimits):
        self.can_strafe, self.limits = can_strafe, limits
        self.state, self.message = D.IDLE, 'idle'
        self.attempt, self.ticket, self.seated = 0, 0, False
        self.clock, self.lost = Stopwatch(), Stopwatch()
        self.last_now: float | None = None
        self.anchor: Pose2D | None = None  # where backing off / undocking started
        self.held_since = 0.0
        self.pending = None  # HookRequest still to hand out
        self.step = ''  # sub-step of the hook phases
        self.zero_once, self.changed, self.charging = False, False, False
        self.centered_for, self.paused = 0.0, ()
        self.notes: list[str] = []
        self.configure(config)

    def configure(self, config: DockingConfig):
        """Apply new parameters (an attempt in progress goes on with them)."""
        self.config = c = config
        self.sign = direction_sign(c.goal)
        self.controller = DockController(
            c.centering, c.approach, self.can_strafe, self.limits, self.sign
        )
        s = c.supervision
        yaw = math.radians(s.stall_yaw_deg)
        self.stall = StallWatch(s.stall_timeout_sec, s.stall_distance, yaw)
        old = getattr(self, 'estimate', None)
        self.estimate = DockEstimate(c.tag.smoothing, c.tag.max_jump, c.tag.max_outliers)
        if old is not None:
            self.estimate.goal, self.estimate.frozen = old.goal, old.frozen

    # ------------------------------------------------------------------ requests

    def dock(self) -> tuple[bool, str]:
        """Start docking from where the robot is (the dock tag should be in view)."""
        if self.state in BUSY:
            return False, f'busy ({self.state.name}): cancel first'
        if self.seated or self.state is D.DOCKED:
            return False, 'sitting on the dock: undock first'
        self.attempt = 1
        self._begin_attempt('docking: looking for the dock tag')
        return True, 'docking started'

    def cancel(self, reason: str = 'cancelled') -> tuple[bool, str]:
        """Stop whatever runs (one zero); hook answers still on their way are ignored."""
        moving = self.state in ATTEMPT or self.step == 'reverse'
        self.ticket += 1
        self.pending, self.step = None, ''
        self._go(D.IDLE, reason)
        self.zero_once = moving
        return True, reason

    # ------------------------------------------------------------------ the tick

    def tick(self, inputs: DockInputs) -> DockDecision:
        """Run one control tick."""
        dt = 0.0 if self.last_now is None else min(max(inputs.now - self.last_now, 0.0), 0.5)
        self.last_now = inputs.now
        self.charging = bool(inputs.charging)
        moving = self.state in ATTEMPT or (self.state is D.UNDOCKING and self.step == 'reverse')
        if moving:
            command = self._moving(inputs, dt)
        else:
            command = self._seated(inputs, dt) if self.state in BUSY | {D.DOCKED} else None
        if command is None and self.zero_once:
            command = Command()
        self.zero_once = False
        hooks, self.pending = self.pending, None
        decision = DockDecision(
            command, self._report(inputs), hooks, self.estimate.goal, self.changed,
            tuple(self.notes),
        )  # fmt: skip
        self.changed, self.notes = False, []
        return decision

    def _moving(self, inputs: DockInputs, dt: float) -> Command:
        """Supervise and run one moving tick; return the command (zero while holding)."""
        if inputs.estop or inputs.remote:
            self._fail('e-stop engaged' if inputs.estop else 'operator took over (REMOTE)')
            return Command()
        sup = self.config.supervision
        pose = inputs.pose
        if pose is None or pose.age > sup.pose_timeout_sec:
            self.held_since += dt
            if self.held_since > sup.lost_timeout_sec:
                reason = 'no odometry pose (odom -> base TF)'
                self._fail(reason) if self.state is D.UNDOCKING else self._attempt_failed(reason)
            return Command()
        self.held_since = 0.0
        if not inputs.armed:
            self.paused = ('disarmed',)
            return Command()  # neutral commands arm navigation in the mux
        self.paused = ()
        self.clock.run(dt)
        self._observe(inputs, dt)
        if self.state is D.UNDOCKING:
            command = self._back_away(pose.pose, self.config.undock.distance, 'undocked')
        else:
            command = getattr(self, f'_{self.state.name.lower()}')(inputs, pose.pose, dt)
        if command is not None and self.stall.stalled(pose.pose, command, dt):
            reason = 'held: commanded but not moving (control field or obstacle?)'
            self._fail(reason) if self.state is D.UNDOCKING else self._attempt_failed(reason)
            return Command()
        return command if command is not None else Command()

    def _observe(self, inputs: DockInputs, dt: float):
        """Fold a fresh tag detection into the dock estimate; run the lost clock."""
        tag = self._fresh_tag(inputs)
        if tag is None:
            self.lost.run(dt)
            return
        self.lost.reset()
        if self.state in (D.SEARCHING, D.CENTERING, D.APPROACHING):
            self.estimate.update(goal_from_tag(tag, self.config.goal))

    def _fresh_tag(self, inputs: DockInputs) -> Pose2D | None:
        tag, pose = inputs.tag, inputs.pose
        if tag is None or tag.age > self.config.tag.timeout_sec or pose is None:
            return None
        if pose.pose.distance(tag.pose) > self.config.tag.max_range:
            return None
        return tag.pose

    def errors(self, pose: Pose2D | None) -> DockErrors:
        """Return the errors of a pose to the dock estimate (NaN while unknown)."""
        if pose is None or self.estimate.goal is None:
            return NO_ERRORS
        return dock_errors(pose, self.estimate.goal, self.sign)

    # ------------------------------------------------------------------ attempt states

    def _searching(self, inputs: DockInputs, pose: Pose2D, dt: float) -> Command:
        if self.estimate.goal is not None and self.lost.elapsed == 0.0:
            self._go(D.CENTERING, 'centring on the dock tag')
        elif self.clock.elapsed > self.config.supervision.search_timeout_sec:
            self._attempt_failed(f'dock tag {self.config.tag.frame} not in view')
        return Command()

    def _centering(self, inputs: DockInputs, pose: Pose2D, dt: float) -> Command:
        if self._tag_lost() or self._timed_out(self.config.centering.timeout_sec, 'centring'):
            return Command()
        e = self.errors(pose)
        if self.controller.centered(e):
            self.centered_for = getattr(self, 'centered_for', 0.0) + dt
            if self.centered_for >= self.config.centering.hold_sec:
                self._go(D.APPROACHING, 'walking to the dock')
                return self.controller.walk(e)
            return Command()
        self.centered_for = 0.0
        return self.controller.center(e)

    def _approaching(self, inputs: DockInputs, pose: Pose2D, dt: float) -> Command:
        e = self.errors(pose)
        if e.remaining <= self.config.approach.blind_distance:
            self.estimate.frozen = True  # the last centimetres on odometry alone
        elif self._tag_lost():
            return Command()
        if self._timed_out(self.config.approach.timeout_sec, 'the approach'):
            return Command()
        if self.controller.arrived(e):
            self._go(D.SETTLING, 'at the dock: settling')
            return Command()
        if self.controller.off_line(e):
            self.centered_for = 0.0
            self._go(D.CENTERING, 'off the approach line: centring again')
            return self.controller.center(e)
        return self.controller.walk(e)

    def _settling(self, inputs: DockInputs, pose: Pose2D, dt: float) -> Command:
        s = self.config.settle
        if self.clock.elapsed < s.time_sec:
            return Command()
        tag = self._fresh_tag(inputs)  # a fresh look beats the frozen estimate
        goal = goal_from_tag(tag, self.config.goal) if tag is not None else self.estimate.goal
        e = dock_errors(pose, goal, self.sign)
        if e.within(s.distance_tolerance, s.lateral_tolerance, math.radians(s.yaw_tolerance_deg)):
            self._start_seated()
        else:
            self._attempt_failed(
                f'not on the dock: remaining {e.remaining:+.3f} m, lateral {e.lateral:+.3f} m,'
                f' yaw {math.degrees(e.yaw):+.1f} deg'
            )
        return Command()

    def _backing_off(self, inputs: DockInputs, pose: Pose2D, dt: float) -> Command:
        return self._back_away(pose, self.config.retry.backoff_distance, None)

    def _back_away(self, pose: Pose2D, distance: float, done: str | None) -> Command:
        """Move straight away from the dock; done: IDLE with that message, None: next attempt."""
        if self.anchor is None:
            self.anchor = pose
        speed = self.config.undock.speed if done else self.config.retry.speed
        limit = self.config.undock.timeout_sec if done else 3.0 * distance / speed + 5.0
        if self.anchor.distance(pose) >= distance or self.clock.elapsed > limit:
            if done is not None:
                self.cancel(done)
            else:
                self._begin_attempt(f'attempt {self.attempt}: looking for the dock tag')
            return Command()
        return self.controller.straight(speed)

    # ------------------------------------------------------------------ transitions

    def _begin_attempt(self, message: str):
        self.estimate.reset()
        self.lost.reset()
        self.centered_for, self.held_since, self.paused = 0.0, 0.0, ()
        self._go(D.SEARCHING, message)

    def _attempt_failed(self, reason: str):
        """Retry after backing off while retries are left, else FAILED."""
        if self.attempt <= self.config.retry.max_retries:
            self.attempt += 1
            self.notes.append(f'attempt failed: {reason}')
            self._go(D.BACKING_OFF, f'{reason}; backing off for attempt {self.attempt}')
            return
        self._fail(reason)

    def _fail(self, reason: str):
        self.ticket += 1
        self.pending, self.step = None, ''
        self.zero_once = True
        self._go(D.FAILED, reason)

    def _tag_lost(self) -> bool:
        if self.lost.elapsed <= self.config.supervision.lost_timeout_sec:
            return False
        self._attempt_failed(f'dock tag {self.config.tag.frame} lost')
        return True

    def _timed_out(self, limit: float, what: str) -> bool:
        if self.clock.elapsed <= limit:
            return False
        self._attempt_failed(f'{what} took longer than {limit:.0f} s')
        return True

    def _go(self, state: D, message: str):
        if state is not self.state:
            self.changed = True
            self.clock.reset()
            self.stall.reset()
            self.anchor = None
        self.state, self.message = state, message
        self.notes.append(f'{state.name}: {message}')

    def _report(self, inputs: DockInputs) -> DockReport:
        tag = inputs.tag
        visible = self._fresh_tag(inputs) is not None
        moving = self.state in ATTEMPT or self.step == 'reverse'
        return DockReport(
            self.state, self.message, getattr(self, 'paused', ()) if moving else (), visible,
            math.inf if tag is None else tag.age,
            self.errors(inputs.pose.pose if inputs.pose else None), self.attempt, self.charging,
        )  # fmt: skip
