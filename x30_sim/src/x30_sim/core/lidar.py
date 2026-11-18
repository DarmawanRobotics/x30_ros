"""The X30's four Livox Mid-360: rings around each tilted dome, ray-cast in a 2.5D world."""

from __future__ import annotations

import math

import numpy as np

from x30_sim.core.config import LidarConfig
from x30_sim.core.world import World, ray_distances
from x30_utils.math import Pose2D

MIN_COS = 1e-6  # a ray this close to vertical has no horizontal reach


def dome_rotation(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """Return the 3x3 rotation of a Mid-360 whose z axis (the dome) points there."""
    a, tilt = math.radians(azimuth_deg), math.radians(90.0 - elevation_deg)
    rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0, 0, 1]])
    ry = np.array(
        [
            [math.cos(tilt), 0.0, math.sin(tilt)],
            [0.0, 1.0, 0.0],
            [-math.sin(tilt), 0.0, math.cos(tilt)],
        ]
    )
    return rz @ ry


def sensor_rays(config: LidarConfig) -> np.ndarray:
    """Return the (rings * azimuth_rays, 3) unit rays of one Mid-360 in its own frame."""
    elevations = np.radians(np.linspace(config.fov_min_deg, config.fov_max_deg, config.rings))
    azimuths = np.linspace(-math.pi, math.pi, config.azimuth_rays, endpoint=False)
    e, a = (v.ravel() for v in np.meshgrid(elevations, azimuths, indexing='ij'))
    return np.column_stack((np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)))


class LidarModel:
    """Ray-cast scans of a world, merged in base_link (level: the body has no pitch or roll)."""

    def __init__(self, config: LidarConfig, base_height: float, rng: np.random.Generator):
        self.config = config
        self.rng = rng
        self.base_height = base_height
        rays = sensor_rays(config)
        self.origins, self.rays = [], []  # per LiDAR: (x, y, z) and (N, 3) rays in base_link
        for x, y, z, azimuth_deg, elevation_deg in config.mount_list:
            self.origins.append(np.array([x, y, z], dtype=float))
            self.rays.append(rays @ dome_rotation(azimuth_deg, elevation_deg).T)

    def ranges(self, world: World, pose: Pose2D, index: int, extra=None) -> np.ndarray:
        """Return the range of every ray of LiDAR index to its first return (inf: none)."""
        origin, rays = self.origins[index], self.rays[index]
        height = self.base_height + origin[2]  # sensor above the floor
        cos_e = np.hypot(rays[:, 0], rays[:, 1])
        sin_e = rays[:, 2]
        heading = pose.yaw + np.arctan2(rays[:, 1], rays[:, 0])
        sensor = pose @ Pose2D(float(origin[0]), float(origin[1]), 0.0)
        dist, heights = ray_distances(world, sensor.x, sensor.y, heading, extra)
        reach = cos_e > MIN_COS
        with np.errstate(divide='ignore', invalid='ignore'):
            r = np.where(reach[:, None], dist / cos_e[:, None], np.inf)
            z = height + r * sin_e[:, None]  # height of the hit above the floor
        visible = np.isfinite(r) & (z >= 0.0) & (z <= heights[None, :])
        nearest = np.where(visible, r, np.inf)
        ranges = nearest.min(axis=1) if nearest.shape[1] else np.full(len(rays), np.inf)
        if self.config.floor_returns:
            down = sin_e < 0.0
            floor = np.full(len(rays), np.inf)
            floor[down] = height / -sin_e[down]
            ranges = np.minimum(ranges, floor)
        return ranges

    def scan(self, world: World, pose: Pose2D, extra=None) -> np.ndarray:
        """Return the (N, 3) float32 points of one merged scan in base_link."""
        clouds = [self._scan_one(world, pose, i, extra) for i in range(len(self.rays))]
        return np.vstack(clouds).astype(np.float32) if clouds else np.empty((0, 3), np.float32)

    def _scan_one(self, world: World, pose: Pose2D, index: int, extra) -> np.ndarray:
        cfg = self.config
        ranges = self.ranges(world, pose, index, extra)
        if cfg.noise_m > 0.0:
            ranges = ranges + self.rng.normal(0.0, cfg.noise_m, ranges.shape)
        keep = np.isfinite(ranges) & (ranges >= cfg.range_min) & (ranges <= cfg.range_max)
        if cfg.point_dropout > 0.0:
            keep &= self.rng.random(ranges.shape) >= cfg.point_dropout
        return self.origins[index] + ranges[keep, None] * self.rays[index][keep]
