"""Which LiDAR points are obstacles: self filter, leveling, height band, ground."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_utils.robot import Footprint


def tilt_rotation(odom_base: np.ndarray) -> np.ndarray:
    """Return the roll/pitch part of an odom->base transform (yaw removed), as 3x3."""
    r = np.asarray(odom_base, dtype=float)[:3, :3]
    yaw = math.atan2(r[1, 0], r[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    unyaw = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return unyaw @ r


def level(points: np.ndarray, tilt: np.ndarray | None) -> np.ndarray:
    """Rotate (N, 3) base_link points into the level frame (tilt None: base assumed level)."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points if tilt is None else points @ np.asarray(tilt, dtype=float).T


def self_mask(points: np.ndarray, footprint: Footprint, margin: float) -> np.ndarray:
    """Return True for base_link points on the robot (inside the footprint grown by margin)."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return footprint.contains(points[:, :2], margin)


def ground_offsets(xy: np.ndarray, heights: np.ndarray, cell: float, max_step: float):
    """Return the local floor height under every point (0 where the floor is not seen)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    heights = np.asarray(heights, dtype=float).reshape(-1)
    if len(heights) == 0:
        return np.zeros(0)
    keys = np.floor(xy / cell).astype(np.int64)
    _, cell_of = np.unique(keys, axis=0, return_inverse=True)
    cell_of = cell_of.reshape(-1)
    lowest = np.full(cell_of.max() + 1, np.inf)
    np.minimum.at(lowest, cell_of, heights)
    floor = np.where(np.abs(lowest) <= max_step, lowest, 0.0)
    return floor[cell_of]


@dataclass(frozen=True)
class Obstacles:
    """Obstacle points in the level frame: (N, 2) xy and their height above the floor (m)."""

    xy: np.ndarray
    height: np.ndarray

    @staticmethod
    def empty() -> Obstacles:
        """No obstacle points."""
        return Obstacles(np.empty((0, 2)), np.empty(0))


def obstacle_points(
    points: np.ndarray,
    footprint: Footprint,
    base_height: float,
    config,
    tilt: np.ndarray | None,
) -> Obstacles:
    """Return the points that can block the robot, leveled and within the height band."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[~self_mask(points, footprint, config.self_margin)]
    leveled = level(points, tilt)
    xy, height = leveled[:, :2], leveled[:, 2] + base_height
    if config.ground_segmentation and len(height):
        height = height - ground_offsets(xy, height, config.ground_cell, config.ground_max_step)
    keep = (height >= config.height_min) & (height <= config.height_max)
    return Obstacles(xy[keep], height[keep])
