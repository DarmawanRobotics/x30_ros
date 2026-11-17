"""Final approach onto an exact x, y, yaw (stop points, the goal, joining the route)."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.config.follower import ApproachConfig
from x30_navigation.core.controllers.base import Command
from x30_utils.math import Pose2D, angle_diff

DIFF_TURN_FIRST = math.radians(20.0)
MIN_YAW_RATE = 0.1


@dataclass(frozen=True)
class ApproachStep:
    """One approach command and how far the robot still is from the target."""

    command: Command
    position_error: float
    yaw_error: float
    done: bool


class FinalApproach:
    """Converge onto a target pose (omni: x, y, yaw together; diff: turn, drive, turn)."""

    def __init__(self, config: ApproachConfig, can_strafe: bool):
        self.config = config
        self.can_strafe = can_strafe
        self.position_reached = False

    def reset(self):
        """Forget the convergence state before a new target."""
        self.position_reached = False

    def step(self, pose: Pose2D, target: Pose2D) -> ApproachStep:
        """Return the command toward target."""
        cfg = self.config
        local = pose.between(target)
        ex, ey = local.x, local.y
        distance = math.hypot(ex, ey)
        yaw_error = angle_diff(target.yaw, pose.yaw)
        if distance < cfg.position_tolerance:
            self.position_reached = True
        elif distance > 2.0 * cfg.position_tolerance:
            self.position_reached = False
        if self.position_reached and abs(yaw_error) < math.radians(cfg.yaw_tolerance_deg):
            return ApproachStep(Command(), distance, yaw_error, True)
        if self.can_strafe:
            command = self._omni(ex, ey, distance, yaw_error)
        else:
            command = self._diff(ex, ey, distance, yaw_error)
        return ApproachStep(command, distance, yaw_error, False)

    def _speed(self, distance: float) -> float:
        cfg = self.config
        return min(cfg.max_speed, max(cfg.min_speed, cfg.kp_xy * distance))

    def _omni(self, ex, ey, distance, yaw_error) -> Command:
        if self.position_reached or distance < 1e-9:
            return Command(0.0, 0.0, self.yaw_rate(yaw_error))
        speed = self._speed(distance)
        return Command(speed * ex / distance, speed * ey / distance, self.yaw_rate(yaw_error))

    def _diff(self, ex, ey, distance, yaw_error) -> Command:
        if self.position_reached:
            return Command(0.0, 0.0, self.yaw_rate(yaw_error))
        bearing = math.atan2(ey, ex)
        direction = 1.0
        if abs(bearing) > math.pi / 2.0:
            bearing, direction = angle_diff(bearing, math.pi), -1.0
        if abs(bearing) > DIFF_TURN_FIRST:
            return Command(0.0, 0.0, self.yaw_rate(bearing))
        speed = direction * self._speed(distance) * math.cos(bearing)
        return Command(speed, 0.0, self.yaw_rate(bearing))

    def yaw_rate(self, error: float) -> float:
        """Proportional yaw rate with a small floor so the last degrees still converge."""
        cfg = self.config
        if abs(error) < math.radians(cfg.yaw_tolerance_deg):
            return 0.0
        rate = max(abs(cfg.kp_yaw * error), MIN_YAW_RATE)
        return math.copysign(min(rate, cfg.max_yaw_rate), error)
