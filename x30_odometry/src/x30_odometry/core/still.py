"""IMU stillness and tilt from the direction of gravity (units do not matter)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class Stillness:
    """Motion statistics of an IMU window."""

    gyro_max: float
    acc_rel_std: float
    tilt: float
    up: np.ndarray = field(repr=False)

    def problems(self, max_gyro: float, max_acc_rel_std: float, max_tilt: float = 0.0) -> list:
        """Return why the window is not still (and level when max_tilt > 0, rad)."""
        out = []
        if self.gyro_max > max_gyro:
            out.append(f'rotating {self.gyro_max:.3f} rad/s')
        if self.acc_rel_std > max_acc_rel_std:
            out.append(f'shaking {self.acc_rel_std * 100.0:.1f} %')
        if max_tilt > 0.0 and self.tilt > max_tilt:
            out.append(f'tilted {math.degrees(self.tilt):.1f} deg')
        return out


def measure(gyro, acc) -> Stillness:
    """Return the stillness of N x 3 gyro (rad/s) and accelerometer samples."""
    gyro, acc = np.asarray(gyro, dtype=float), np.asarray(acc, dtype=float)
    gyro_max = float(np.max(np.linalg.norm(gyro, axis=1)))
    norms = np.linalg.norm(acc, axis=1)
    mean_norm = float(np.mean(norms))
    mean_acc = np.mean(acc, axis=0)
    length = float(np.linalg.norm(mean_acc))
    if mean_norm <= 1e-9 or length <= 1e-9:
        return Stillness(gyro_max, math.inf, math.pi, UP.copy())
    up = mean_acc / length
    return Stillness(gyro_max, float(np.std(norms)) / mean_norm, angle_to_up(up), up)


def angle_to_up(vector) -> float:
    """Return the angle (rad) between a vector and +z."""
    v = np.asarray(vector, dtype=float)
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        return math.pi
    return math.acos(max(-1.0, min(1.0, float(v[2]) / n)))


class StillWindow:
    """The IMU samples of the last window_sec seconds."""

    def __init__(self, window_sec: float, min_samples: int = 20):
        self.window_sec, self.min_samples = window_sec, min_samples
        self.samples: deque = deque()

    def add(self, t: float, gyro, acc):
        """Add a sample received at t (s); non-finite samples are ignored."""
        gyro, acc = np.asarray(gyro, dtype=float), np.asarray(acc, dtype=float)
        if not (np.all(np.isfinite(gyro)) and np.all(np.isfinite(acc))):
            return
        if self.samples and t < self.samples[-1][0]:
            self.samples.clear()  # time went back
        self.samples.append((t, gyro, acc))
        while self.samples and t - self.samples[0][0] > self.window_sec:
            self.samples.popleft()

    def clear(self):
        """Forget every sample."""
        self.samples.clear()

    def full(self, now: float) -> bool:
        """Return True when the window covers window_sec up to now with enough samples."""
        if len(self.samples) < self.min_samples:
            return False
        newest, oldest = self.samples[-1][0], self.samples[0][0]
        fresh = now - newest <= max(0.2, 0.1 * self.window_sec)
        return fresh and newest - oldest >= 0.9 * self.window_sec

    def measure(self, now: float) -> Stillness | None:
        """Return the stillness of the window, or None while it is not full."""
        if not self.full(now):
            return None
        gyro = np.array([s[1] for s in self.samples])
        acc = np.array([s[2] for s in self.samples])
        return measure(gyro, acc)


@dataclass(frozen=True)
class GateDecision:
    """What wait_still does now."""

    release: bool
    forced: bool
    reason: str
    stillness: Stillness | None = None


class StillGate:
    """Holds the estimator start until the IMU is still and level, or until the timeout."""

    def __init__(self, config, start: float):
        self.config = config
        self.start = start
        self.window = StillWindow(config.still_sec, config.min_samples)

    def add(self, t: float, gyro, acc):
        """Add an IMU sample received at t (s)."""
        self.window.add(t, gyro, acc)

    def decide(self, now: float) -> GateDecision:
        """Return whether to start the estimator now and why (not)."""
        c = self.config
        stillness = self.window.measure(now)
        reason = 'no IMU data yet' if not self.window.samples else 'collecting IMU samples'
        if stillness is not None:
            problems = stillness.problems(
                c.max_gyro_rad_s, c.max_acc_rel_std, math.radians(c.max_tilt_deg)
            )
            if not problems:
                return GateDecision(True, False, 'still and level', stillness)
            reason = ', '.join(problems)
        if c.timeout_sec > 0.0 and now - self.start > c.timeout_sec:
            return GateDecision(True, True, reason, stillness)
        return GateDecision(False, False, reason, stillness)
