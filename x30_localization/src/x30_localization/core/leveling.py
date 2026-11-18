"""Gravity alignment of odom: the IMU's up direction at start gives the base roll and pitch."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from x30_utils.math import rotation_aligning

UP = np.array([0.0, 0.0, 1.0])
MIN_SAMPLES = 10


@dataclass(frozen=True)
class StillLimits:
    """When the IMU window counts as standing still."""

    window_sec: float = 1.0
    max_rate: float = math.radians(3.0)
    max_accel_spread: float = 0.03
    max_tilt: float = math.radians(20.0)


class StillWindow:
    """The IMU samples of the last window_sec seconds."""

    def __init__(self, limits: StillLimits | None = None):
        self.limits = limits or StillLimits()
        self.samples: deque[tuple[float, np.ndarray, np.ndarray]] = deque()

    def add(self, stamp: float, accel, gyro):
        """Add one IMU sample (non-finite samples are ignored)."""
        accel, gyro = np.asarray(accel, dtype=float), np.asarray(gyro, dtype=float)
        if not (np.all(np.isfinite(accel)) and np.all(np.isfinite(gyro))):
            return
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()  # time jumped back (bag loop, sim restart)
        self.samples.append((stamp, accel, gyro))
        while self.samples and stamp - self.samples[0][0] > self.limits.window_sec:
            self.samples.popleft()

    def mean_specific_force(self) -> tuple[np.ndarray | None, str]:
        """Return (mean accelerometer vector, '') while still, else (None, reason)."""
        if len(self.samples) < MIN_SAMPLES:
            return None, f'{len(self.samples)} IMU samples in the window'
        span = self.samples[-1][0] - self.samples[0][0]
        if span < 0.8 * self.limits.window_sec:
            return None, f'IMU window {span:.2f} s of {self.limits.window_sec:.2f} s'
        accel = np.array([s[1] for s in self.samples])
        gyro = np.array([s[2] for s in self.samples])
        rate = float(np.linalg.norm(gyro, axis=1).max())
        if rate > self.limits.max_rate:
            return None, f'moving: {math.degrees(rate):.1f} deg/s'
        mean = accel.mean(axis=0)
        norm = float(np.linalg.norm(mean))
        if norm < 1e-6:
            return None, 'accelerometer reads zero'
        spread = float(np.linalg.norm(accel - mean, axis=1).std()) / norm
        spread = max(spread, float(np.linalg.norm(accel.std(axis=0))) / norm)
        if spread > self.limits.max_accel_spread:
            return None, f'shaking: accelerometer spread {spread:.3f}'
        return mean, ''


def level_rotation(
    specific_force_imu, r_base_imu: np.ndarray, r_init_body: np.ndarray, max_tilt: float
) -> tuple[np.ndarray | None, str]:
    """Return (3x3 roll/pitch of base at start, '') or (None, reason when implausible)."""
    f = np.asarray(specific_force_imu, dtype=float)
    up_start = np.asarray(r_base_imu)[:3, :3] @ np.asarray(r_init_body)[:3, :3] @ f
    up_start /= np.linalg.norm(up_start)
    tilt = math.acos(max(-1.0, min(1.0, float(up_start @ UP))))
    if tilt > max_tilt:
        return None, (
            f'start tilt {math.degrees(tilt):.1f} deg above {math.degrees(max_tilt):.0f} deg '
            '(robot on a slope or wrong IMU frame?)'
        )
    level = rotation_aligning(up_start, UP)
    yaw = math.atan2(level[1, 0], level[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    unyaw = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return unyaw @ level, ''  # roll and pitch only: odom x stays the heading at start


def tilt_angle(rotation: np.ndarray) -> float:
    """Return the angle (rad) between the rotated z axis and z."""
    return math.acos(max(-1.0, min(1.0, float(np.asarray(rotation)[2, 2]))))


class Leveler:
    """Levels a (re)started odometry once still, or gives up after timeout_sec (0: never)."""

    def __init__(self, limits: StillLimits, timeout_sec: float, enabled: bool = True):
        self.window = StillWindow(limits)
        self.timeout_sec = timeout_sec
        self.enabled = enabled
        self.since: float | None = None

    def restart(self):
        """Forget the IMU window: the odometry starts over at a new pose."""
        self.window.samples.clear()
        self.since = None

    def add(self, stamp: float, accel, gyro):
        """Add an IMU sample."""
        if self.enabled:
            self.window.add(stamp, accel, gyro)

    def level(self, r_base_imu, r_init_body, now: float) -> tuple[np.ndarray | None, bool, str]:
        """Return (level rotation or None, decided, reason)."""
        if not self.enabled:
            return None, True, 'leveling disabled'
        if self.since is None:
            self.since = now
        force, reason = self.window.mean_specific_force()
        if force is not None:
            level, reason = level_rotation(
                force, r_base_imu, r_init_body, self.window.limits.max_tilt
            )
            if level is not None:
                return level, True, ''
        if self.timeout_sec > 0.0 and now - self.since >= self.timeout_sec:
            return None, True, f'not leveled after {self.timeout_sec:.0f} s: {reason}'
        return None, False, reason
