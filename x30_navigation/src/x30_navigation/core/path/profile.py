"""Speed profile along a route: curvature limit, braking into stops, accelerating out of them."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_navigation.core.path.route import Route
from x30_utils.safety import stopping_distance, stopping_speed


@dataclass(frozen=True)
class ProfileLimits:
    """Speeds (m/s), accelerations (m/s^2) and the reaction time (s) of the profile."""

    cruise: float
    min_speed: float
    approach: float
    lateral_accel: float
    accel: float
    decel: float
    reaction_time: float = 0.0


def speed_profile(route: Route, limits: ProfileLimits) -> np.ndarray:
    """Return the speed limit at every route point: curvature, stops, braking, acceleration."""
    kappa = np.abs(route.curvature)
    with np.errstate(divide='ignore'):
        curve = np.where(kappa > 1e-6, np.sqrt(limits.lateral_accel / kappa), np.inf)
    v = np.clip(curve, limits.min_speed, limits.cruise)
    hard = np.union1d(route.stops, [len(route) - 1])
    v[hard] = np.minimum(v[hard], limits.approach)
    ds = np.diff(route.s)
    _backward(v, ds, limits.decel)
    _stopping(v, route.s, hard, limits)
    v[0] = min(v[0], max(limits.min_speed, limits.approach))
    _forward(v, ds, limits.accel)
    return v


def speed_at(route: Route, profile: np.ndarray, s: float) -> float:
    """Return the profile speed at arc length s."""
    return float(np.interp(s, route.s, profile))


def _backward(v: np.ndarray, ds: np.ndarray, decel: float):
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], float(np.sqrt(v[i + 1] ** 2 + 2.0 * decel * ds[i])))


def _forward(v: np.ndarray, ds: np.ndarray, accel: float):
    for i in range(len(v) - 1):
        v[i + 1] = min(v[i + 1], float(np.sqrt(v[i] ** 2 + 2.0 * accel * ds[i])))


def _stopping(v: np.ndarray, s: np.ndarray, hard: np.ndarray, limits: ProfileLimits):
    """Cap speeds before every hard point so the braking model reaches its speed there."""
    previous = 0
    for k in hard:
        head = stopping_distance(float(v[k]), limits.decel, 0.0)
        for i in range(previous, int(k)):
            room = float(s[k] - s[i]) + head
            v[i] = min(v[i], stopping_speed(room, limits.decel, limits.reaction_time))
        previous = int(k)
