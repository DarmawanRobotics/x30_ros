"""Scan -> grid: height band in the level map frame, self filter, correction and jump gates."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from x30_mapping.core import matching
from x30_mapping.core.config import MapperConfig
from x30_mapping.core.grid import LogOddsGrid
from x30_utils.math import Pose2D, angle_diff, inverse, transform_points
from x30_utils.robot import RobotProfile


class Outcome(Enum):
    """What happened to a scan."""

    INTEGRATED = 0
    EMPTY = 1  # no usable point after the filters
    CORRECTION = 2  # map->odom changed around the scan
    JUMP = 3  # the pose jumped: integration holds
    HELD = 4  # still holding after a jump, the scan disagrees with the map


@dataclass(frozen=True)
class Scan:
    """One scan with the transforms at its stamp."""

    points: np.ndarray  # (N, 3) in the sensor frame
    stamp: float  # s
    base_sensor: np.ndarray  # 4x4 base <- sensor (static mount)
    map_odom: np.ndarray  # 4x4 map <- odom at the stamp
    odom_sensor: np.ndarray  # 4x4 odom <- sensor at the stamp
    map_odom_window: tuple = ()  # map <- odom before the stamp and at the newest time


@dataclass(frozen=True)
class ScanResult:
    """The outcome of one scan."""

    outcome: Outcome
    used: int = 0
    message: str = ''
    correction: Pose2D | None = None  # refinement applied (map frame), None: TF pose used

    @property
    def rejected(self) -> bool:
        """True for scans skipped by a gate."""
        return self.outcome in (Outcome.CORRECTION, Outcome.JUMP, Outcome.HELD)


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per voxel (the first one), or all points when voxel <= 0."""
    if voxel <= 0.0 or points.shape[0] < 2:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def correction_in_flight(samples, tol_m: float, tol_rad: float) -> bool:
    """Return True when the map->odom samples (4x4) differ by more than the tolerances."""
    poses = [Pose2D.from_matrix(m) for m in samples]
    first = poses[0]
    return any(
        first.distance(p) > tol_m or abs(angle_diff(p.yaw, first.yaw)) > tol_rad for p in poses[1:]
    )


class ScanIntegrator:
    """Filter, gate and integrate scans into a LogOddsGrid."""

    def __init__(self, config: MapperConfig, robot: RobotProfile, grid: LogOddsGrid):
        self.config = config
        self.robot = robot
        self.grid = grid
        limits = robot.limits
        self.max_speed = float(np.hypot(limits.max_vx, limits.max_vy))
        self.max_wz = float(limits.max_wz)
        self.reset(grid)

    def reset(self, grid: LogOddsGrid):
        """Start over on a grid (new session): no previous pose, not holding."""
        self.grid = grid
        self.previous: tuple[Pose2D, float] | None = None
        self.holding = ''
        self.rejected = 0

    def process(self, scan: Scan) -> ScanResult:
        """Integrate one scan when the gates let it through."""
        result = self._process(scan)
        if result.rejected:
            self.rejected += 1
        return result

    def _process(self, scan: Scan) -> ScanResult:
        gate = self.config.gate
        samples = (scan.map_odom, *scan.map_odom_window)
        if len(samples) > 1 and correction_in_flight(
            samples, gate.correction_tol_m, np.radians(gate.correction_tol_deg)
        ):
            return ScanResult(Outcome.CORRECTION, message='map->odom correction in flight')
        map_sensor = scan.map_odom @ scan.odom_sensor
        pose = Pose2D.from_matrix(map_sensor)
        jumped = self._jumped(pose, scan.stamp)
        self.previous = (pose, scan.stamp)
        points = self._obstacles(scan, map_sensor)
        if jumped:
            self.holding = jumped
        if self.holding:
            if not self._agrees(points, pose):
                outcome = Outcome.JUMP if jumped else Outcome.HELD
                return ScanResult(outcome, message=self.holding)
            self.holding = ''
        if len(points) == 0:
            return ScanResult(Outcome.EMPTY, message='no obstacle point in the band')
        return self._integrate(points, pose)

    def _jumped(self, pose: Pose2D, stamp: float) -> str:
        """Return why the pose jumped since the previous scan, or ''."""
        if self.previous is None:
            return ''
        last, last_stamp = self.previous
        max_m, max_rad = matching.jump_limits(
            self.config.gate, self.max_speed, self.max_wz, stamp - last_stamp
        )
        moved, turned = last.distance(pose), abs(angle_diff(pose.yaw, last.yaw))
        if moved <= max_m and turned <= max_rad:
            return ''
        return (
            f'pose jumped {moved:.2f} m / {np.degrees(turned):.0f} deg: holding until a scan '
            'agrees with the map (relocalize, or restart mapping)'
        )

    def _obstacles(self, scan: Scan, map_sensor: np.ndarray) -> np.ndarray:
        """Return the (N, 2) map-frame obstacle points of a scan."""
        cfg = self.config.scan
        points = np.asarray(scan.points, dtype=np.float64).reshape(-1, 3)
        points = voxel_downsample(points[np.isfinite(points).all(axis=1)], cfg.voxel)
        if len(points) == 0:
            return np.empty((0, 2))
        in_base = transform_points(scan.base_sensor, points)
        in_map = transform_points(map_sensor, points)
        map_base = map_sensor @ inverse(scan.base_sensor)
        height = in_map[:, 2] - (map_base[2, 3] - self.robot.base_height)
        own = self.robot.footprint.contains(in_base[:, :2], cfg.self_margin)
        keep = (height >= cfg.height_min) & (height <= cfg.height_max) & ~own
        return in_map[keep, :2]

    def _agrees(self, points: np.ndarray, pose: Pose2D) -> bool:
        if len(points) < 10:
            return False
        radius = self.config.scan.max_range
        score = matching.agreement(self.grid, points, (pose.x, pose.y), radius)
        return score is not None and score >= self.config.gate.resume_score

    def _integrate(self, points: np.ndarray, pose: Pose2D) -> ScanResult:
        cfg = self.config
        sensor, correction, note = pose, None, ''
        if cfg.refine.enabled:
            matched, note = matching.refine(
                self.grid, points, pose, cfg.refine, cfg.scan.max_range
            )
            if matched is not None:
                points = matched.apply(pose.inverse().apply(points))
                correction, sensor = pose.between(matched), matched
        used = self.grid.integrate(
            (sensor.x, sensor.y),
            points,
            cfg.scan.angular_bins,
            cfg.scan.max_range,
            cfg.scan.min_range,
        )
        if used == 0:
            return ScanResult(Outcome.EMPTY, message='every hit outside the grid or the range')
        return ScanResult(Outcome.INTEGRATED, used, note, correction)
