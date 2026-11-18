"""From one cloud (and a depth camera's) to the front and safety sector ranges (pure, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from x30_perception.core.blindness import check_blind
from x30_perception.core.config import PerceptionConfig
from x30_perception.core.depth import DepthCloud
from x30_perception.core.filtering import Obstacles, obstacle_points, self_mask
from x30_perception.core.livox import noise_mask
from x30_perception.core.memory import NearField, ObstacleMemory
from x30_perception.core.sectors import SectorLayout, kth_ranges
from x30_utils.math import Pose2D, transform_points
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class Frame:
    """One cloud and what is known about the robot when it was taken."""

    xyz: np.ndarray
    sensor_to_base: np.ndarray
    stamp: float
    tags: np.ndarray | None = None
    tilt: np.ndarray | None = None
    odom_base: Pose2D | None = None


@dataclass(frozen=True)
class ScanResult:
    """Both scans' ranges, the obstacle points, why BLOCKED, the depth camera's ranges."""

    front: np.ndarray
    safety: np.ndarray
    obstacles: Obstacles
    remembered: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    blocked_reason: str = ''
    depth: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))


class SectorPipeline:
    """LiDAR and depth camera to sector scans: filters, blindness, band, binning, memories."""

    def __init__(self, config: PerceptionConfig, robot: RobotProfile):
        self.config = config
        self.robot = robot
        f, s = config.front, config.safety
        self.front = SectorLayout.front(
            math.radians(f.fov_deg), f.num_sectors, f.range_min, f.range_max
        )
        self.safety = SectorLayout.circle(s.num_sectors, s.range_min, s.range_max)
        m = config.memory
        self.memory = ObstacleMemory(m.memory_sec, m.radius, m.resolution, m.max_cells)
        seconds = config.depth.memory_sec
        self.depth_memory = ObstacleMemory(seconds, m.radius, m.resolution, m.max_cells)
        self._last_pose: Pose2D | None = None
        self._last_stamp: float | None = None
        noise = config.filter
        self._noise_fields = tuple(
            name
            for name, on in (
                ('adhesion', noise.drop_adhesion_noise),
                ('particle', noise.drop_particle_noise),
                ('other', noise.drop_other_noise),
            )
            if on
        )

    def blocked(self, reason: str) -> ScanResult:
        """Both scans with every sector BLOCKED (blind, no TF, no data)."""
        return ScanResult(
            self.front.blocked(), self.safety.blocked(), Obstacles.empty(), blocked_reason=reason
        )

    def process(self, frame: Frame, depth: DepthCloud | None = None) -> ScanResult:
        """Return the scans of one cloud; BLOCKED when the cloud or the leveling is unusable."""
        xyz = np.asarray(frame.xyz, dtype=float).reshape(-1, 3)
        if frame.tags is not None and self._noise_fields:
            xyz = xyz[~noise_mask(frame.tags, self._noise_fields, self.config.filter.noise_level)]
        base = transform_points(frame.sensor_to_base, xyz)
        on_robot = self_mask(base, self.robot.footprint, self.config.filter.self_margin)
        b = self.config.blind
        blind = check_blind(xyz[~on_robot], b.min_points, b.regions, b.min_region_points)
        if blind.blind:
            return self.blocked(f'LiDAR blind: {blind.reason}')
        if self.config.filter.level_source == 'tf' and frame.tilt is None:
            return self.blocked('no roll/pitch of the base (odom -> base TF)')
        d = self.config.depth
        if d.enabled and d.required and (depth is None or depth.reason):
            return self.blocked(f'depth camera: {"no cloud" if depth is None else depth.reason}')
        obstacles = obstacle_points(
            base[~on_robot], self.robot.footprint, self.robot.base_height,
            self.config.filter, frame.tilt,
        )  # fmt: skip
        front = kth_ranges(self.front, obstacles.xy, self.config.front.k)
        safety, confirmed = self._safety(obstacles)
        camera = np.empty(0, dtype=np.float32)
        if depth is not None and not depth.reason:
            seen = depth.obstacles.xy
            front = np.minimum(front, kth_ranges(self.front, seen, 1))
            camera = kth_ranges(self.safety, seen, 1)
            safety = np.minimum(safety, camera)
        remembered = self._remember(frame, confirmed, depth)
        if len(remembered):
            safety = np.minimum(safety, kth_ranges(self.safety, remembered, 1))
        return ScanResult(front, safety, obstacles, remembered, depth=camera)

    def _safety(self, obstacles: Obstacles) -> tuple[np.ndarray, Obstacles]:
        """Return the safety ranges and the obstacle points in sectors confirmed by k points."""
        k = self.config.safety.k
        rows, idx, _ = self.safety.locate(obstacles.xy)
        counts = np.bincount(idx, minlength=self.safety.num_sectors)
        keep = rows[counts[idx] >= k]
        confirmed = Obstacles(obstacles.xy[keep], obstacles.height[keep])
        return kth_ranges(self.safety, obstacles.xy, k), confirmed

    def _remember(
        self, frame: Frame, confirmed: Obstacles, depth: DepthCloud | None
    ) -> np.ndarray:
        """Update both memories; return the remembered points to merge into the safety scan."""
        odom, stamp = frame.odom_base, frame.stamp
        lidar = self.config.memory.enabled
        camera = self.config.depth.enabled and self.config.depth.memory_sec > 0.0
        if odom is None or not self.config.safety.enabled or not (lidar or camera):
            self.memory.clear()
            self.depth_memory.clear()
            self._last_pose = None
            return np.empty((0, 2))
        if self._jumped(odom, stamp):
            self.memory.clear()
            self.depth_memory.clear()
        self._last_pose, self._last_stamp = odom, stamp
        view = self.view(frame.sensor_to_base)
        out = [np.empty((0, 2))]
        if lidar:
            self.memory.update(odom, confirmed.xy, confirmed.height, stamp)
            out.append(self.memory.hidden(odom, stamp, view))
        if camera:
            seeing = depth.view if depth is not None and not depth.reason else None
            if seeing is not None:
                self.depth_memory.forget(odom, seeing.contains)  # it sees there now
            if depth is not None and not depth.reason:
                self.depth_memory.update(odom, depth.obstacles.xy, depth.obstacles.height, stamp)
            hidden = self.depth_memory.hidden(odom, stamp, view)
            out.append(hidden if seeing is None else hidden[~seeing.contains(hidden)])
        return np.vstack(out)

    def view(self, sensor_to_base: np.ndarray) -> NearField:
        """Return what the lowest beam of the sensor sees from its mount."""
        return NearField.of_mount(
            sensor_to_base, self.robot.base_height, math.radians(self.config.memory.lower_fov_deg)
        )

    def _jumped(self, pose: Pose2D, stamp: float) -> bool:
        """Return True when odom moved faster than the robot can (odometry reset or jump)."""
        if self._last_pose is None or self._last_stamp is None:
            return False
        dt = stamp - self._last_stamp
        if dt < 0.0:
            return True
        limits = self.robot.limits
        speed = max(limits.max_vx, limits.max_vy)
        reach = 2.0 * speed * dt + self.config.memory.max_jump
        turn = 2.0 * limits.max_wz * dt + math.radians(30.0)
        delta = self._last_pose.between(pose)
        return math.hypot(delta.x, delta.y) > reach or abs(delta.yaw) > turn
