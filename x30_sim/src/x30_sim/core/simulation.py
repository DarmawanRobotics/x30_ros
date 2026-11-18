"""The simulated X30 on one clock: body, odometries, LiDARs, IMU, battery, dock and scenario."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_sim.core.body import Body, BodyLimits, Twist
from x30_sim.core.config import SimConfig
from x30_sim.core.dock import DockSpec, DockStation, TagCamera
from x30_sim.core.imu import imu_sample
from x30_sim.core.latency import CommandQueue, PoseHistory, sensor_stamp
from x30_sim.core.lidar import LidarModel
from x30_sim.core.odometry import DriftingOdometry
from x30_sim.core.scenario import Scenario, make_rngs
from x30_sim.core.world import World
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class OdometrySample:
    """odom -> base_link of the leg odometry and the body twist at stamp."""

    stamp: float
    odom_base: Pose2D
    twist: Twist


@dataclass(frozen=True)
class ScanSample:
    """A merged LiDAR scan in base_link at stamp."""

    stamp: float
    points: np.ndarray


class Simulation:
    """Everything the simulated robot does, without ROS."""

    def __init__(
        self,
        config: SimConfig,
        robot: RobotProfile,
        world: World,
        scenario: Scenario | None = None,
    ):
        self.scenario = scenario or Scenario()
        self.config = cfg = self.scenario.apply(config)
        self.world = world
        lidar_rng, odom_rng, self._imu_rng, leg_rng, camera_rng = make_rngs(cfg.seed)
        self.camera = TagCamera(cfg.camera, world, camera_rng)
        self.station = DockStation(DockSpec.from_dict(world.dock), world)
        self.body = Body(
            world.start, BodyLimits.from_robot(robot, cfg.max_dt), Body.footprint_box(robot), world
        )
        self.commands = CommandQueue(cfg.latency.command_sec, cfg.cmd_timeout_sec, Twist())
        self.lio_drift = DriftingOdometry(cfg.odometry, odom_rng, world.start)  # FAST-LIO
        self.leg_drift = DriftingOdometry(cfg.leg_odometry, leg_rng, world.start)  # x30_driver
        self.lidar = LidarModel(cfg.lidar, robot.base_height, lidar_rng)
        self.truth = PoseHistory()
        self.odom = PoseHistory()
        self.leg_odom = PoseHistory()
        self.dynamic = np.empty((0, 4))  # circles from sim/obstacles
        self.battery = cfg.battery.start
        self.start_time: float | None = None
        self._elapsed = 0.0
        self.restarts = 0  # odometry restarts so far

    def elapsed(self, now: float) -> float:
        """Return the seconds since the first step (the scenario clock)."""
        return 0.0 if self.start_time is None else now - self.start_time

    def obstacles(self, now: float) -> np.ndarray:
        """Return the dynamic circles (sim/obstacles and the scenario's) at now."""
        return np.vstack([self.dynamic, self.scenario.circles_at(self.elapsed(now))])

    def set_obstacles(self, xy: np.ndarray, radius: float, height: float):
        """Replace the sim/obstacles circles by circles at the (N, 2) map positions."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        self.dynamic = np.column_stack(
            (xy, np.full(len(xy), radius), np.full(len(xy), height))
        ).reshape(-1, 4)

    def command(self, now: float, twist: Twist):
        """Take a velocity command received at now."""
        self.commands.push(now, twist)

    def step(self, now: float) -> float:
        """Advance the body, the odometries and the battery to now; return the integrated dt."""
        if self.start_time is None:
            self.start_time = now
        previous, self._elapsed = self._elapsed, self.elapsed(now)
        command = Twist() if self.station.sitting else self.commands.active(now)
        dt = self.body.step(now, command, self.obstacles(now))
        if self.scenario.restart_due(previous, self._elapsed):
            self.lio_drift.restart(self.body.pose)
            self.odom = PoseHistory()
            self.restarts += 1
        self.truth.add(now, self.body.pose)
        self.odom.add(now, self.lio_drift.update(self.body.pose))
        self.leg_odom.add(now, self.leg_drift.update(self.body.pose))
        battery = self.config.battery
        rate = battery.charge_per_hour if self.station.charging else 0.0
        rate -= battery.drain_per_hour
        self.battery = min(1.0, max(0.0, self.battery + rate * dt / 3600.0))
        return dt

    def silent(self, sensor: str, now: float) -> bool:
        """Return True while a scenario dropout silences sensor."""
        return self.scenario.silent(sensor, self.elapsed(now))

    def leg_odometry(self, now: float) -> OdometrySample | None:
        """Return the driver's leg odometry published at now, None while silent."""
        if self.silent('leg_odometry', now):
            return None
        stamp = sensor_stamp(now, self.config.latency.odometry_sec)
        pose = self.leg_odom.at(stamp)
        return None if pose is None else OdometrySample(stamp, pose, self.body.twist)

    def scan(self, now: float) -> ScanSample | None:
        """Return the scan published at now (cast at the pose of now - latency), None if silent."""
        if self.silent('lidar', now):
            return None
        stamp = sensor_stamp(now, self.config.latency.lidar_sec)
        pose = self.truth.at(stamp)
        if pose is None:
            return None
        return ScanSample(stamp, self.lidar.scan(self.world, pose, self.obstacles(now)))

    def lio(self, stamp: float, now: float) -> Pose2D | None:
        """Return FAST-LIO's odom -> base_link for the scan of stamp, None while it is silent."""
        return None if self.silent('odometry', now) else self.odom.at(stamp)

    def tag_detections(self, now: float) -> dict[str, np.ndarray]:
        """Return the dock tags in view now (base_link -> tag); none in an `all` dropout."""
        if self.silent('all', now):
            return {}
        return self.camera.detections(self.body.pose)

    def imu(self, now: float) -> tuple[np.ndarray, np.ndarray] | None:
        """Return (accelerometer m/s^2, gyro rad/s) at now, None while silent."""
        if self.silent('imu', now):
            return None
        xy = (self.config.imu.xyz[0], self.config.imu.xyz[1])
        return imu_sample(self.body.twist, self.body.accel, xy, self._imu_rng)
