"""The static bridge from the LiDAR-inertial odometry into map -> odom -> base_link."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_utils.math import compose, inverse


@dataclass(frozen=True)
class Bridge:
    """The two static transforms of the bridge (4x4)."""

    odom_init: np.ndarray
    body_base: np.ndarray
    leveled: bool

    def odom_base(self, init_body: np.ndarray) -> np.ndarray:
        """Return odom->base_link for a LIO pose camera_init->body."""
        return compose(self.odom_init, init_body, self.body_base)


def make_bridge(m_base_imu: np.ndarray, level: np.ndarray | None = None) -> Bridge:
    """Return the bridge for the URDF base_link->imu and the level rotation (None: unleveled)."""
    # odom->camera_init = L * T_base_imu, body->base_link = inv(T_base_imu): odom is base_link
    # at start, leveled by L (map -> odom -> camera_init -> body -> base_link)
    m_level = np.eye(4)
    if level is not None:
        m_level[:3, :3] = np.asarray(level)[:3, :3]
    return Bridge(compose(m_level, m_base_imu), inverse(m_base_imu), level is not None)
