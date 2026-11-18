"""A pure Follower on a kinematic robot with a simulated 360 deg scan, for the unit tests."""

from __future__ import annotations

import math

import numpy as np
from sim import step

from x30_navigation.core.avoidance import AvoidanceOut
from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers import CONTROLLERS, ControllerLimits
from x30_navigation.core.follower.fsm import Follower
from x30_navigation.core.follower.state import FollowerState, Inputs, PoseInput, ScanInput
from x30_navigation.core.scan import SectorScanData
from x30_utils.math import Pose2D
from x30_utils.robot import Footprint, Limits, RobotProfile

ROBOT = RobotProfile(
    footprint=Footprint(front=0.4, rear=0.4, half_width=0.28),
    limits=Limits(max_vx=0.8, max_vy=0.4, max_wz=1.0, max_accel=0.5, max_decel=0.6),
)
S = FollowerState


class Rig:
    """Drive a Follower: obstacles are map-frame points seen by a perfect 360 deg scan."""

    def __init__(self, route, config=None, robot=ROBOT, pose=None, controller='holonomic'):
        self.config = config or FollowerConfig(require_localization=False)
        self.robot = robot
        limits = robot.limits
        self.follower = Follower(
            self.config,
            robot,
            CONTROLLERS.create(
                controller, None, ControllerLimits(limits.max_vx, limits.max_vy, limits.max_wz)
            ),
        )
        self.follower.set_route(route)
        self.pose = pose or Pose2D(
            float(route.xy[0, 0]), float(route.xy[0, 1]), float(route.yaw[0])
        )
        self.obstacles = np.zeros((0, 2))
        self.blind = False
        self.scan_age = 0.0
        self.pose_age = 0.0
        self.avoidance = AvoidanceOut()
        self.avoidance_age = 0.0
        self.have_pose = True
        self.t = 0.0
        self.dt = 0.05
        self.decisions = []
        self.commands = []
        self.sectors = 0  # > 0: the scan is a sector scan of that many sectors, as on the robot
        self.emergency_zone = None  # (distance, cone_deg): avoidance's emergency from obstacles

    def scan(self) -> ScanInput:
        local = (
            self.pose.inverse().apply(self.obstacles) if len(self.obstacles) else np.zeros((0, 2))
        )
        if not self.sectors:
            return ScanInput(local, self.blind, self.scan_age)
        data = sector_scan(local, self.sectors)
        edges, centres = data.points(), data.points(edges=False)
        return ScanInput(edges.points, self.blind, self.scan_age, centres.points)

    def frontal(self) -> bool:
        """Return True when an obstacle is in emergency_zone (obstacle_avoidance_node's cone)."""
        if self.emergency_zone is None or not len(self.obstacles):
            return False
        distance, cone_deg = self.emergency_zone
        local = self.pose.inverse().apply(self.obstacles)
        near = np.hypot(local[:, 0], local[:, 1]) < distance
        ahead = np.abs(np.arctan2(local[:, 1], local[:, 0])) < math.radians(cone_deg) / 2.0
        return bool(np.any(near & ahead))

    def inputs(self) -> Inputs:
        pose = PoseInput(self.pose, self.pose_age) if self.have_pose else None
        last = self.follower.shaper.last
        avoidance = self.avoidance
        if self.emergency_zone is not None:
            avoidance = AvoidanceOut(emergency=self.frontal())
        return Inputs(self.t, self.dt, pose, self.scan(), avoidance, self.avoidance_age, last)

    def tick(self, move: bool = True):
        decision = self.follower.tick(self.inputs())
        self.decisions.append(decision)
        if decision.command is not None:
            self.commands.append(decision.command)
            if move:
                c = decision.command
                self.pose = step(self.pose, c.vx, c.vy, c.wz, self.dt)
        self.t += self.dt
        return decision

    def run(self, seconds: float, until=None):
        for _ in range(int(seconds / self.dt)):
            decision = self.tick()
            if until is not None and until(decision):
                return decision
        return self.decisions[-1]

    @property
    def state(self) -> FollowerState:
        return self.decisions[-1].report.state if self.decisions else self.follower.state

    def events(self):
        return [(e.kind.name, e.index) for d in self.decisions for e in d.events]


def sector_scan(points: np.ndarray, sectors: int, range_max: float = 8.0) -> SectorScanData:
    """Nearest range per sector of points (robot frame), 360 deg from -pi."""
    inc = 2.0 * math.pi / sectors
    ranges = np.full(sectors, range_max)
    if len(points):
        i = np.floor((np.arctan2(points[:, 1], points[:, 0]) + math.pi) / inc).astype(int)
        np.minimum.at(ranges, i % sectors, np.hypot(points[:, 0], points[:, 1]))
    return SectorScanData.of(-math.pi, inc, range_max, ranges)


def front_scan(points: np.ndarray, sectors: int = 32, range_max: float = 8.0) -> SectorScanData:
    """Nearest range per sector of points (robot frame) over 180 deg ahead (perception front)."""
    inc = math.pi / sectors
    ranges = np.full(sectors, range_max)
    if len(points):
        i = np.floor((np.arctan2(points[:, 1], points[:, 0]) + math.pi / 2.0) / inc).astype(int)
        ok = (i >= 0) & (i < sectors)
        np.minimum.at(ranges, i[ok], np.hypot(points[ok, 0], points[ok, 1]))
    return SectorScanData.of(-math.pi / 2.0, inc, range_max, ranges)


def person(x, y, r=0.25, n=16):
    """Points on the outline of someone standing at x, y."""
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.column_stack((x + r * np.cos(a), y + r * np.sin(a)))


def is_zero(command) -> bool:
    return command is not None and command.vx == command.vy == command.wz == 0.0
