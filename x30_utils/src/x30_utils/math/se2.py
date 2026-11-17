"""Planar poses in SE(2)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.math.angles import wrap_angle
from x30_utils.math.transforms import planar_transform, yaw_of


@dataclass(frozen=True)
class Pose2D:
    """A planar pose: position (m) and heading (rad)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __matmul__(self, other: Pose2D) -> Pose2D:
        """Compose: self then other expressed in self."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> Pose2D:
        """Return the inverse transform."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, wrap_angle(-self.yaw))

    def between(self, other: Pose2D) -> Pose2D:
        """Return other expressed in the frame of self."""
        return self.inverse() @ other

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Transform (N, 2) points from this frame into the parent frame."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return points @ np.array([[c, s], [-s, c]]) + (self.x, self.y)

    def distance(self, other: Pose2D) -> float:
        """Return the Euclidean distance between the positions."""
        return math.hypot(other.x - self.x, other.y - self.y)

    def to_array(self) -> np.ndarray:
        """Return [x, y, yaw]."""
        return np.array([self.x, self.y, self.yaw])

    def to_matrix(self) -> np.ndarray:
        """Return the 4x4 transform."""
        return planar_transform(self.x, self.y, self.yaw)

    @staticmethod
    def from_array(values) -> Pose2D:
        """Build from [x, y, yaw]."""
        return Pose2D(float(values[0]), float(values[1]), wrap_angle(float(values[2])))

    @staticmethod
    def from_matrix(m: np.ndarray) -> Pose2D:
        """Build from the planar part of a 4x4 transform."""
        return Pose2D(float(m[0, 3]), float(m[1, 3]), yaw_of(m))

    def is_finite(self) -> bool:
        """Return True when every component is finite."""
        return math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.yaw)


def adjoint(pose: Pose2D) -> np.ndarray:
    """Return the 3x3 adjoint of a pose, mapping tangent vectors through it."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.array([[c, -s, pose.y], [s, c, -pose.x], [0.0, 0.0, 1.0]])
