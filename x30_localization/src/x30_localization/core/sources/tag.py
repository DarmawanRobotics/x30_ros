"""Fiducial source: map->base from a tag of known pose; noise grows with distance and angle."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_localization.core.sources.base import Measurement, Source
from x30_utils.math import Pose2D, compose, inverse

MIN_DISTANCE_M = 0.3


@dataclass(frozen=True)
class TagNoise:
    """Noise model of a detection and its validity limits."""

    sigma_xy_at_1m: float = 0.02
    sigma_yaw_at_1m: float = math.radians(1.0)
    max_distance_m: float = 4.0
    max_view_angle: float = math.radians(60.0)


def map_base_from_tag(m_map_tag: np.ndarray, m_base_cam: np.ndarray, m_cam_tag: np.ndarray):
    """Return the 4x4 map->base implied by seeing a tag whose map pose is known."""
    return compose(m_map_tag, inverse(compose(m_base_cam, m_cam_tag)))


def map_tag_from_base(m_map_base: np.ndarray, m_base_cam: np.ndarray, m_cam_tag: np.ndarray):
    """Return the 4x4 map->tag of a detection while the robot is at map->base (recording)."""
    return compose(m_map_base, m_base_cam, m_cam_tag)


def view_angle(m_cam_tag: np.ndarray) -> float:
    """Return the angle (rad) between the line of sight and the tag normal (0: head-on)."""
    position = m_cam_tag[:3, 3]
    distance = float(np.linalg.norm(position))
    if distance < 1e-9:
        return 0.0
    normal = m_cam_tag[:3, 2]  # the tag z axis, either sign
    cos = abs(float(np.dot(position / distance, normal)))
    return math.acos(min(1.0, cos))


def tag_covariance(robot_to_tag_xy, distance: float, angle: float, noise: TagNoise):
    """Return the 3x3 (x, y, yaw) covariance of map->base from one detection."""
    d = max(distance, MIN_DISTANCE_M)
    oblique = 1.0 / max(math.cos(angle), 0.1)
    sigma_p = noise.sigma_xy_at_1m * d * d * oblique
    sigma_yaw = noise.sigma_yaw_at_1m * d * oblique
    # a heading error dpsi rotates the robot about the tag: dp = -S v dpsi, S = [[0,-1],[1,0]]
    lever = np.array([robot_to_tag_xy[1], -robot_to_tag_xy[0]])
    cov = np.zeros((3, 3))
    cov[:2, :2] = sigma_p**2 * np.eye(2) + sigma_yaw**2 * np.outer(lever, lever)
    cov[:2, 2] = cov[2, :2] = sigma_yaw**2 * lever
    cov[2, 2] = sigma_yaw**2
    return cov


class TagSource(Source):
    """Measurements of map->base from detections of known tags."""

    name = 'tag'

    def __init__(self, noise: TagNoise | None = None):
        self.noise = noise or TagNoise()

    def measure(
        self,
        frame: str,
        m_map_tag: np.ndarray,
        m_base_cam: np.ndarray,
        m_cam_tag: np.ndarray,
        stamp: float,
    ) -> Measurement | str:
        """Return the measurement of one detection, or why it is not usable."""
        distance = float(np.linalg.norm(m_cam_tag[:3, 3]))
        angle = view_angle(m_cam_tag)
        if not math.isfinite(distance) or distance > self.noise.max_distance_m:
            return f'{frame} too far ({distance:.1f} m)'
        if angle > self.noise.max_view_angle:
            return f'{frame} seen too obliquely ({math.degrees(angle):.0f} deg)'
        m_map_base = map_base_from_tag(m_map_tag, m_base_cam, m_cam_tag)
        z = Pose2D.from_matrix(m_map_base)
        robot_to_tag = m_map_tag[:2, 3] - m_map_base[:2, 3]
        cov = tag_covariance(robot_to_tag, distance, angle, self.noise)
        return Measurement(z, cov, stamp, f'tag:{frame}')
