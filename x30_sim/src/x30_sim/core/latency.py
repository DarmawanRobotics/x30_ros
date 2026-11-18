"""Sensor and actuation latency: stamps show the state of now - L, commands act L late."""

from __future__ import annotations

from collections import deque
from typing import Generic, TypeVar

from x30_utils.math import Pose2D, wrap_angle

T = TypeVar('T')


def sensor_stamp(now: float, latency_sec: float) -> float:
    """Return the stamp (and state time) of a measurement published at now."""
    return now - latency_sec


class CommandQueue(Generic[T]):
    """Velocity commands delayed by latency_sec; a stale command is replaced by zero."""

    def __init__(self, latency_sec: float, timeout_sec: float, zero: T):
        self.latency_sec = latency_sec
        self.timeout_sec = timeout_sec
        self.zero = zero
        self._queue: deque[tuple[float, T]] = deque()
        self._active: tuple[float, T] | None = None

    def push(self, now: float, command: T):
        """Add a command received at now."""
        if self._queue and now < self._queue[-1][0]:
            self._queue.clear()  # the clock jumped back
        self._queue.append((now, command))

    def active(self, now: float) -> T:
        """Return the command acting at now (zero when none or stale)."""
        effective = now - self.latency_sec
        while self._queue and self._queue[0][0] <= effective:
            self._active = self._queue.popleft()
        if self._active is None or effective - self._active[0] > self.timeout_sec:
            return self.zero
        return self._active[1]


class PoseHistory:
    """Recent (time, pose) samples with linear interpolation."""

    def __init__(self, horizon_sec: float = 2.0):
        self.horizon_sec = horizon_sec
        self._samples: deque[tuple[float, Pose2D]] = deque()

    def add(self, t: float, pose: Pose2D):
        """Record the pose at t (a clock jump back clears the history)."""
        if self._samples and t < self._samples[-1][0]:
            self._samples.clear()
        self._samples.append((t, pose))
        while len(self._samples) > 2 and t - self._samples[1][0] > self.horizon_sec:
            self._samples.popleft()

    def at(self, t: float) -> Pose2D | None:
        """Return the pose at t (clamped to the recorded span), None when empty."""
        if not self._samples:
            return None
        if t <= self._samples[0][0]:
            return self._samples[0][1]
        samples = list(self._samples)
        for i in range(len(samples) - 1, 0, -1):  # newest first: lookups are recent
            (t0, p0), (t1, p1) = samples[i - 1], samples[i]
            if t0 <= t <= t1:
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                return Pose2D(
                    p0.x + a * (p1.x - p0.x),
                    p0.y + a * (p1.y - p0.y),
                    wrap_angle(p0.yaw + a * wrap_angle(p1.yaw - p0.yaw)),
                )
        return self._samples[-1][1]
