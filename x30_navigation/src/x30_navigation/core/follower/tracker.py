"""Tracking one route: progress, speed profile, path shift and the path controller."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from x30_navigation.core.avoidance.base import AvoidanceOut
from x30_navigation.core.config.follower import FollowerConfig, RegulatorConfig
from x30_navigation.core.controllers.base import Bias, Command, PathController, TrackRequest
from x30_navigation.core.follower.state import ScanInput
from x30_navigation.core.path.profile import ProfileLimits, speed_at, speed_profile
from x30_navigation.core.path.progress import RouteProgress
from x30_navigation.core.path.route import Route
from x30_navigation.core.shift.path_shift import HOLD_SPEED, PathShift, ShiftOutput
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class TrackOut:
    """One tracking step."""

    command: Command
    speed: float
    shift: ShiftOutput | None = None
    debug: dict = field(default_factory=dict)


class SpeedRegulator:
    """Slow down while turning harder than a reference yaw rate (signed error, clamped sum)."""

    def __init__(self, config: RegulatorConfig):
        self.config = config
        self.integral = 0.0

    def reset(self):
        """Forget the integral (new route)."""
        self.integral = 0.0

    def scale(self, yaw_rate: float, dt: float) -> float:
        """Return the 0..1 speed factor for a commanded yaw rate."""
        cfg = self.config
        error = abs(yaw_rate) - cfg.reference_yaw_rate
        limit = cfg.integral_limit
        self.integral = max(-limit, min(limit, self.integral + error * max(dt, 0.0)))
        correction = cfg.kp * error + cfg.ki * self.integral
        return max(cfg.min_scale, min(1.0, 1.0 - correction))


def profile_limits(config: FollowerConfig, robot: RobotProfile) -> ProfileLimits:
    """Return the speed profile limits: the follower's, never above the robot's."""
    speed, limits = config.speed, robot.limits
    cruise = min(speed.cruise, limits.max_vx if limits.max_vx > 0 else speed.cruise)
    return ProfileLimits(
        cruise=cruise,
        min_speed=min(speed.min_speed, cruise),
        approach=min(speed.approach_speed, cruise),
        lateral_accel=speed.lateral_accel,
        accel=min(speed.accel, limits.max_accel),
        decel=min(speed.decel, limits.max_decel),
        reaction_time=limits.reaction_time,
    )


class Tracker:
    """Follow one route with a controller."""

    def __init__(
        self,
        route: Route,
        robot: RobotProfile,
        config: FollowerConfig,
        controller: PathController,
    ):
        self.route = route
        self.robot = robot
        self.config = config
        self.controller = controller
        self.progress = RouteProgress(route)
        self.profile = speed_profile(route, profile_limits(config, robot))
        self.shift = PathShift(route, robot, config.path_shift)
        self.regulator = SpeedRegulator(config.regulator)

    @property
    def shifting(self) -> bool:
        """True when obstacles are passed by sliding sideways.

        Also with a controller that avoids by itself: it keeps the clearance, the shift picks the
        side it passes on.
        """
        return self.config.avoidance_mode == 'path_shift' and self.controller.strafes

    @property
    def reacting(self) -> bool:
        """True when reactive avoidance biases the controller (not one that avoids by itself)."""
        return self.config.avoidance_mode == 'reactive' and not self.controller.avoids

    @property
    def offset(self) -> float:
        """The lateral offset the robot is meant to be at (m, +left)."""
        return self.shift.offset if self.shifting else 0.0

    def restart(self, s: float):
        """Put the progress at s, back on the route, controller state cleared."""
        self.progress.reset(s)
        self.shift.reset()
        self.regulator.reset()
        self.controller.reset()

    def advance(self, pose: Pose2D, dt: float) -> float:
        """Advance the progress (capped per tick) and return it."""
        limits = self.robot.limits
        cap = 2.0 * max(limits.max_vx, limits.max_vy) * max(dt, 0.0) + 0.05
        return self.progress.update(pose, self.offset, cap)

    def lateral(self, pose: Pose2D) -> float:
        """Signed distance of the robot from the route at the progress (m, +left)."""
        return self.progress.lateral(pose)

    def step(
        self,
        pose: Pose2D,
        scan: ScanInput,
        avoidance: AvoidanceOut | None,
        now: float,
        dt: float,
        slow: bool,
        hold_route: bool,
        forward: bool = True,
        last: Command | None = None,
    ) -> TrackOut:
        """Return the target command; hold_route keeps the robot on the route (near stops)."""
        s = self.progress.s
        speed = speed_at(self.route, self.profile, s) if forward else 0.0
        if slow:
            speed = min(speed, self.config.speed.min_speed)
        shift = None
        if self.shifting:
            points = pose.apply(scan.points) if len(scan.points) else np.zeros((0, 2))
            lateral = self.lateral(pose)
            held = last is not None and self.along(pose, last) < HOLD_SPEED
            shift = self.shift.update(
                s, lateral, points, scan.blocked, now, dt, hold_route, forward, held
            )
            speed = min(speed, shift.speed_limit)
        bias, scale, push = Bias(), 1.0, 0.0
        if self.reacting and avoidance is not None:
            bias = Bias(avoidance.heading_offset, avoidance.yaw_rate_bias)
            scale, push = avoidance.velocity_scale, avoidance.lateral_bias
        if shift is not None and shift.slide:
            return self._slide(pose, s, dt, shift, scan)
        request = TrackRequest(self.route, s, pose, speed, dt, self.offset, bias, scan.points)
        command = self.controller.update(request).scaled(scale)
        if self.robot.can_strafe and push:
            command = Command(command.vx, command.vy + push, command.wz)
        if self.config.regulator.enabled and not self.controller.strafes:
            command = command.scaled(self.regulator.scale(command.wz, dt))
        if not math.isfinite(speed):
            speed = 0.0
        return TrackOut(command, speed, shift, dict(self.controller.debug))

    def _slide(self, pose: Pose2D, s: float, dt: float, shift: ShiftOutput, scan: ScanInput):
        """Forward held by path shift: slide sideways toward the offset, keep the heading."""
        command = self.slide_command(pose, shift.slide, dt, scan)
        return TrackOut(command, abs(shift.slide), shift, dict(self.controller.debug))

    def slide_command(self, pose: Pose2D, slide: float, dt: float, scan: ScanInput) -> Command:
        """Return a pure sideways slide (m/s, +left of the route) with the heading control."""
        s = self.progress.s
        request = TrackRequest(self.route, s, pose, 0.0, dt, self.offset, Bias(), scan.points)
        wz = self.controller.update(request).wz
        normal = self.route.segment_yaw[self.route.segment_at(s)] + math.pi / 2.0 - pose.yaw
        return Command(slide * math.cos(normal), slide * math.sin(normal), wz)

    def held_slide(self, pose: Pose2D, dt: float, scan: ScanInput) -> Command | None:
        """Return the slide toward the committed offset for a command the guard held (or None)."""
        if not self.shifting:
            return None
        slide = self.shift.slide(self.lateral(pose))
        return self.slide_command(pose, slide, dt, scan) if slide else None

    def along(self, pose: Pose2D, command: Command) -> float:
        """Return the speed of a robot-frame command along the route at the progress (m/s)."""
        yaw = self.route.segment_yaw[self.route.segment_at(self.progress.s)] - pose.yaw
        return command.vx * math.cos(yaw) + command.vy * math.sin(yaw)

    def held_parallel(self, pose: Pose2D, target: Command) -> Command | None:
        """Return the part of a held command along the route (or None): keep the offset, go on."""
        if not (self.robot.can_strafe and self.controller.strafes):
            return None
        yaw = self.route.segment_yaw[self.route.segment_at(self.progress.s)] - pose.yaw
        c, s = math.cos(yaw), math.sin(yaw)
        along = target.vx * c + target.vy * s
        if along <= 1e-3:
            return None
        return Command(along * c, along * s, target.wz)
