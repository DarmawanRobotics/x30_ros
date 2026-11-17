"""Backing up along the driven path when the robot gets no further along the route for a while."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.state import FollowerState, Inputs
from x30_utils.math import Pose2D, angle_diff

S = FollowerState
SPACING_M = 0.05  # trail poses at least this far apart
JUMP_M = 0.5  # a pose this far from the last one starts the trail over (relocalized, carried)
KEEP_M = 10.0  # trail length kept
MIN_WAY_M = 0.2  # a shorter trail: straight back along the heading instead
LOOK_AHEAD_M = 0.3
DONE_M = 0.03
SLOW_M = 0.3  # slower over the last metres of the way back
KP_YAW = 1.5  # 1/s
MAX_YAW_RATE = 0.5  # rad/s
REAR_HOLD_SEC = 3.0  # held this long by the guard (the rear): the back-up ends
TIMEOUT_FACTOR, TIMEOUT_EXTRA_SEC = 3.0, 5.0
BACK_MARGIN_M = 0.5  # the progress is searched this much further back than the way back


class Trail:
    """Poses the robot drove through while following, oldest first."""

    def __init__(self):
        self.poses: deque = deque()
        self.length = 0.0

    def clear(self):
        """Forget the trail."""
        self.poses.clear()
        self.length = 0.0

    def add(self, pose: Pose2D):
        """Append pose once it is SPACING_M from the last one; a jump starts the trail over."""
        if self.poses:
            step = _distance(self.poses[-1], pose)
            if step > JUMP_M:
                self.clear()
            elif step < SPACING_M:
                return
            else:
                self.length += step
        self.poses.append(pose)
        while len(self.poses) > 2 and self.length - _distance(*self._oldest()) > KEEP_M:
            self.length -= _distance(*self._oldest())
            self.poses.popleft()

    def way_back(self, pose: Pose2D, distance: float) -> WayBack:
        """Return the way from pose back along the trail, at most distance long."""
        out, travelled = [pose], 0.0
        for previous in reversed(self.poses):
            step = _distance(out[-1], previous)
            if step < 1e-6:
                continue
            if travelled + step >= distance:
                out.append(_between(out[-1], previous, (distance - travelled) / step))
                break
            out.append(previous)
            travelled += step
        return WayBack(out)

    def rewind(self, distance: float):
        """Drop the newest distance of the trail (the part driven back)."""
        while len(self.poses) > 1 and distance > 0.0:
            step = _distance(self.poses[-2], self.poses[-1])
            if step > distance:
                self.poses[-1] = _between(self.poses[-1], self.poses[-2], distance / step)
                self.length -= distance
                return
            self.poses.pop()
            self.length -= step
            distance -= step

    def _oldest(self) -> tuple[Pose2D, Pose2D]:
        return self.poses[0], self.poses[1]


class WayBack:
    """A polyline to drive backward along: from the robot (first) to where it backs up to."""

    def __init__(self, poses: list[Pose2D]):
        self.xy = np.array([[p.x, p.y] for p in poses], dtype=float).reshape(-1, 2)
        self.yaw = np.array([p.yaw for p in poses], dtype=float)
        steps = np.hypot(*np.diff(self.xy, axis=0).T) if len(poses) > 1 else np.zeros(0)
        self.s = np.concatenate(([0.0], np.cumsum(steps)))
        self.length = float(self.s[-1])

    @classmethod
    def straight(cls, pose: Pose2D, distance: float) -> WayBack:
        """Return the way straight back along the robot's heading."""
        end = Pose2D(
            pose.x - distance * math.cos(pose.yaw),
            pose.y - distance * math.sin(pose.yaw),
            pose.yaw,
        )
        return cls([pose, end])

    def project(self, pose: Pose2D, done: float) -> tuple[float, float]:
        """Return (how far along the way the robot is, at least done; its distance from it)."""
        if self.length <= 0.0:
            return done, 0.0
        a, seg = self.xy[:-1], np.diff(self.xy, axis=0)
        lengths = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)
        rel = np.array([pose.x, pose.y]) - a
        t = np.clip((rel * seg).sum(axis=1) / lengths**2, 0.0, 1.0)
        gap = np.hypot(*(rel - t[:, None] * seg).T)
        k = int(np.argmin(gap))
        return max(done, float(self.s[k] + t[k] * lengths[k])), float(gap[k])

    def at(self, s: float) -> tuple[float, float, float]:
        """Return x, y and the driven heading at arc length s."""
        s = min(max(s, 0.0), self.length)
        k = min(int(np.searchsorted(self.s, s, side='right')) - 1, len(self.s) - 2)
        k = max(k, 0)
        span = self.s[k + 1] - self.s[k]
        t = (s - self.s[k]) / span if span > 0.0 else 0.0
        x, y = self.xy[k] + t * (self.xy[k + 1] - self.xy[k])
        return (
            float(x),
            float(y),
            float(self.yaw[k] + t * angle_diff(self.yaw[k + 1], self.yaw[k])),
        )

    def command(self, pose: Pose2D, done: float, speed: float, strafes: bool) -> Command:
        """Return the command toward the look-ahead point behind, keeping the driven heading."""
        x, y, yaw = self.at(done + LOOK_AHEAD_M)
        local = pose.inverse().apply(np.array([[x, y]]))[0]
        remaining = self.length - done
        v = min(speed, max(0.3 * speed, speed * remaining / SLOW_M))
        reach = math.hypot(local[0], local[1])
        if reach < 1e-6:
            return Command()
        if strafes:
            wz = _clamp(KP_YAW * angle_diff(yaw, pose.yaw), MAX_YAW_RATE)
            return Command(v * local[0] / reach, v * local[1] / reach, wz)
        curvature = 2.0 * local[1] / (reach * reach)  # pure pursuit, driven backward
        return Command(-v, 0.0, _clamp(-v * curvature, MAX_YAW_RATE))


class StuckWatch:
    """How long the progress along the route stayed within min_progress of where it was."""

    def __init__(self):
        self.anchor: float | None = None
        self.seconds = 0.0

    def reset(self):
        """Forget the time (not following, paused, a new start)."""
        self.anchor, self.seconds = None, 0.0

    def update(self, s: float, dt: float, min_progress: float) -> float:
        """Return the time without progress (s); min_progress further along starts it over."""
        if self.anchor is None or s > self.anchor + min_progress:
            self.anchor, self.seconds = s, 0.0
        else:
            self.seconds += max(dt, 0.0)
        return self.seconds


@dataclass
class Backtrack:
    """The trail, the time without progress and the back-up going on, if any."""

    trail: Trail = field(default_factory=Trail)
    watch: StuckWatch = field(default_factory=StuckWatch)
    way: WayBack | None = None
    done: float = 0.0
    seconds: float = 0.0
    held: float = 0.0
    attempts: int = 0
    stuck_at: float | None = None  # progress where the attempts started (past it: count anew)

    @property
    def active(self) -> bool:
        """True while backing up."""
        return self.way is not None

    def begin(self, way: WayBack, s: float):
        """Start backing up along way; the robot is stuck at progress s."""
        self.way, self.done, self.seconds, self.held = way, 0.0, 0.0, 0.0
        self.attempts += 1
        if self.stuck_at is None:
            self.stuck_at = s
        self.watch.reset()

    def end(self):
        """Stop backing up (the trail driven back is dropped)."""
        self.trail.rewind(self.done)
        self.way = None

    def reset(self):
        """Forget everything (a new start, route or lap)."""
        self.way = None
        self.trail.clear()
        self.watch.reset()
        self.attempts, self.stuck_at = 0, None


class BacktrackSteps:
    """Follower steps of backing up when stuck (a mixin of Follower, like Phases)."""

    def _watch_stuck(self, inputs: Inputs, pose: Pose2D | None):
        """Record the trail and start backing up after backtrack.after_sec without progress."""
        bt, cfg = self.backtrack, self.config.backtrack
        if self.phase is not S.FOLLOWING or bt.active or pose is None:
            if not bt.active:
                bt.watch.reset()
            return
        bt.trail.add(pose)
        s = self.tracker.progress.s
        if bt.stuck_at is not None and s > bt.stuck_at + cfg.min_progress:
            bt.attempts, bt.stuck_at = 0, None  # got past where it was stuck
        stuck = bt.watch.update(s, inputs.dt, cfg.min_progress)
        if not cfg.enabled or stuck < cfg.after_sec:
            return
        if bt.attempts >= cfg.max_attempts:
            self.block_note = f'stuck: backed up {bt.attempts} times, waiting for a clear way'
            return
        way = bt.trail.way_back(pose, cfg.distance)
        if way.length < MIN_WAY_M:
            way = WayBack.straight(pose, cfg.distance)
        bt.begin(way, s)
        self.detour.end()
        self._set_phase(S.JOINING)

    def _backtracking(self, inputs: Inputs, pose: Pose2D):
        """Back up along the way until its end, a blocked rear or the timeout; then follow."""
        bt, cfg = self.backtrack, self.config.backtrack
        bt.seconds += inputs.dt
        bt.done, off = bt.way.project(pose, bt.done)
        command = bt.way.command(pose, bt.done, cfg.speed, self.robot.can_strafe)
        rear = self.guard.limit(command, inputs.scan.guard_points, inputs.scan.blocked)
        bt.held = bt.held + inputs.dt if rear.stalled else 0.0
        timeout = TIMEOUT_FACTOR * bt.way.length / cfg.speed + TIMEOUT_EXTRA_SEC
        note = ''
        if off > JUMP_M:
            note = f'moved {off:.2f} m away from the way back'
        elif bt.done >= bt.way.length - DONE_M:
            note = f'backed up {bt.done:.2f} m along the driven path'
        elif bt.held > REAR_HOLD_SEC:
            note = f'rear blocked after {bt.done:.2f} m'
        elif bt.seconds > timeout:
            note = f'back-up timeout after {bt.done:.2f} m'
        attempt = f'attempt {bt.attempts}/{cfg.max_attempts}'
        if note:
            self._end_backtrack(pose)
            return Command(), f'{note}, following again ({attempt})'
        return command, (
            f'stuck {cfg.after_sec:.0f} s: backing up along the driven path'
            f' {bt.done:.2f} / {bt.way.length:.2f} m ({attempt})'
        )

    def _end_backtrack(self, pose: Pose2D):
        """Follow again from where the backing up ended, at the robot's offset from the route."""
        bt, tracker = self.backtrack, self.tracker
        bt.end()
        s = tracker.progress.back(pose, self.config.backtrack.distance + BACK_MARGIN_M)
        lateral = tracker.lateral(pose)
        tracker.restart(s)
        if tracker.shifting:
            shift = tracker.shift
            shift.offset = shift.target = lateral
            shift.searching = abs(lateral) > self.config.path_shift.max_offset + 1e-9
        self._set_phase(S.FOLLOWING)


def _distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def _between(a: Pose2D, b: Pose2D, t: float) -> Pose2D:
    """Return the pose a fraction t of the way from a to b."""
    return Pose2D(
        a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * angle_diff(b.yaw, a.yaw)
    )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
