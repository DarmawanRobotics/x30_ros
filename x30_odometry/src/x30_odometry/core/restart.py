"""Jump and restart detection on consecutive estimator poses (yaw alone never counts)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.math import yaw_from_quat


@dataclass(frozen=True)
class PoseSample:
    """One estimator pose: stamp (s), position (m) and orientation (x, y, z, w)."""

    stamp: float
    position: np.ndarray
    orientation: tuple

    @property
    def yaw(self) -> float:
        """Yaw (rad)."""
        return yaw_from_quat(*self.orientation)


def step_of(prev: PoseSample, cur: PoseSample) -> float:
    """Return the translation (m) between two poses."""
    return float(np.linalg.norm(cur.position - prev.position))


def is_jump(step_m: float, dt: float, max_speed_mps: float, jump) -> bool:
    """Return True when a step is longer than min_step_m and faster than the robot can move."""
    if step_m <= jump.min_step_m:
        return False
    return step_m > jump.speed_factor * max_speed_mps * max(dt, 0.0)


def at_origin(pose: PoseSample, restart) -> bool:
    """Return True when a pose is where a fresh estimator starts."""
    near = float(np.linalg.norm(pose.position)) <= restart.origin_radius_m
    return near and abs(pose.yaw) <= math.radians(restart.origin_yaw_deg)


def restart_reason(prev: PoseSample | None, cur: PoseSample, jumped: bool, restart) -> str:
    """Return why cur starts a new estimator frame, or '' when it continues the old one."""
    if prev is None:
        return ''
    dt = cur.stamp - prev.stamp
    if dt < 0.0:
        return f'time went back {-dt:.2f} s'
    if not at_origin(cur, restart):
        return ''
    if dt > restart.gap_sec:
        return f'back at the origin after {dt:.1f} s without odometry'
    if jumped:
        return f'jumped {step_of(prev, cur):.2f} m back to the origin'
    return ''
