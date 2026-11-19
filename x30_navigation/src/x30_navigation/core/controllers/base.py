"""Path controllers: one interface, every implementation registered by name."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
import math
from typing import ClassVar

import numpy as np

from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D
from x30_utils.plugins import Registry


@dataclass(frozen=True)
class Bias:
    """Reactive avoidance applied on top of path tracking."""

    heading_offset: float = 0.0
    yaw_rate_bias: float = 0.0


@dataclass(frozen=True)
class Command:
    """Body velocity: forward and sideways (m/s), yaw rate (rad/s)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def scaled(self, factor: float) -> Command:
        """Return the command with the translation scaled (the yaw rate kept)."""
        return Command(self.vx * factor, self.vy * factor, self.wz)


DEFAULT_FOOTPRINT = ((0.4, 0.28), (-0.4, 0.28), (-0.4, -0.28), (0.4, -0.28))
"""The robot profile's default footprint corners (m, base_link), counter-clockwise."""


@dataclass(frozen=True)
class ControllerLimits:
    """What the robot can do and the body it moves, from the robot profile; the control period."""

    max_vx: float = 0.8
    max_vy: float = 0.4
    max_wz: float = 1.0
    period_sec: float = 0.05
    max_accel: float = 0.5
    max_yaw_accel: float = 1.5
    footprint: tuple = DEFAULT_FOOTPRINT


@dataclass(frozen=True)
class TrackRequest:
    """Everything a controller needs for one update."""

    route: Route
    s: float
    pose: Pose2D
    speed: float
    dt: float
    offset: float = 0.0
    bias: Bias = Bias()
    obstacles: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


@dataclass(frozen=True)
class NoConfig:
    """A controller without parameters."""


class PathController(ABC):
    """Base class of path-following controllers."""

    Config: ClassVar[type] = NoConfig
    strafes: ClassVar[bool] = False
    avoids: ClassVar[bool] = False
    """True: the controller keeps its own clearance from the obstacle points, so reactive
    avoidance stands by; path shift still picks the side it passes on."""

    def __init__(self, config=None, limits: ControllerLimits | None = None):
        self.config = config if config is not None else self.Config()
        self.limits = limits or ControllerLimits()
        self.debug: dict = {}
        self.warning = ''  # a standing problem the follower logs (solves too slow), '' when none

    def reset(self):
        """Forget internal state (new route, restart)."""
        self.debug = {}

    def update(self, request: TrackRequest) -> Command:
        """Return the command for this tick, avoidance biases applied."""
        offset = request.bias.heading_offset
        if self.strafes or offset == 0.0:
            command = self.compute(request)
        else:
            # a steering controller holds the robot's heading at the path heading; rotating
            # its view of the heading by -offset turns the direction of travel by +offset
            pose = request.pose
            turned = Pose2D(pose.x, pose.y, pose.yaw - offset)
            command = self.compute(replace(request, pose=turned))
        if self.strafes and offset != 0.0:
            command = steered(command, offset)
        return replace(command, wz=command.wz + request.bias.yaw_rate_bias)

    @abstractmethod
    def compute(self, request: TrackRequest) -> Command:
        """Return the nominal command toward the (offset) route."""

    def clamp_wz(self, wz: float) -> float:
        """Limit a yaw rate to the robot's."""
        limit = self.limits.max_wz
        return max(-limit, min(limit, wz)) if math.isfinite(wz) else 0.0


CONTROLLERS: Registry[PathController] = Registry('x30.controllers', PathController)


def steered(command: Command, offset: float) -> Command:
    """Return a strafing command that drives the avoidance heading (robot frame), same speed."""
    speed = math.hypot(command.vx, command.vy)
    travel = math.pi if command.vx < 0.0 else 0.0
    heading = travel + offset
    return Command(speed * math.cos(heading), speed * math.sin(heading), command.wz)


def carrot(request: TrackRequest, distance: float) -> np.ndarray:
    """Return the point distance metres ahead on the offset route."""
    return request.route.shifted_point(request.s + distance, request.offset)


def to_body(pose: Pose2D, point: np.ndarray) -> tuple[float, float]:
    """Return a map point in the robot frame."""
    dx, dy = point[0] - pose.x, point[1] - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy
