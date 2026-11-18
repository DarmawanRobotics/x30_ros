from __future__ import annotations

import math

import numpy as np

TAU = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % TAU - math.pi


def wrap_angles(angles: np.ndarray) -> np.ndarray:
    """Wrap an array of angles to [-pi, pi)."""
    return (np.asarray(angles) + np.pi) % TAU - np.pi


def angle_diff(a: float, b: float) -> float:
    """Return a - b wrapped to [-pi, pi)."""
    return wrap_angle(a - b)


def mean_angle(angles, weights=None) -> float:
    """Return the circular mean of angles."""
    angles = np.asarray(angles, dtype=float)
    w = np.ones_like(angles) if weights is None else np.asarray(weights, dtype=float)
    return math.atan2(float(np.sum(w * np.sin(angles))), float(np.sum(w * np.cos(angles))))


def circular_std(angles) -> float:
    """Return the circular standard deviation of angles."""
    angles = np.asarray(angles, dtype=float)
    r = math.hypot(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
    return math.sqrt(max(0.0, -2.0 * math.log(max(r, 1e-12))))
