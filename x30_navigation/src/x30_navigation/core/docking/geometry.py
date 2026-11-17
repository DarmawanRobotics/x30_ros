"""Dock geometry: the tag as a planar pose, the docked pose in front of it and the errors to it."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.config.docking import GoalConfig
from x30_navigation.core.separation import protective_distance
from x30_utils.math import Pose2D, wrap_angle
from x30_utils.robot import RobotProfile

AXES = {'x': 0, 'y': 1, 'z': 2}
MIN_HORIZONTAL = 0.5  # |normal| in the floor plane below this: the tag is not on a wall


@dataclass(frozen=True)
class DockErrors:
    """The robot relative to the docked pose."""

    remaining: float
    lateral: float
    yaw: float

    def within(self, distance: float, lateral: float, yaw: float) -> bool:
        """Return True when every error is within its tolerance."""
        r, s, a = abs(self.remaining), abs(self.lateral), abs(self.yaw)
        return r <= distance and s <= lateral and a <= yaw


def tag_pose(tag_in_frame: np.ndarray, robot_xy, axis: str = 'z') -> Pose2D | None:
    """Return the tag as a planar pose (yaw: normal toward the robot) from its 4x4 transform."""
    m = np.asarray(tag_in_frame, dtype=float)
    if m.shape != (4, 4) or not np.all(np.isfinite(m)):
        return None
    normal = m[:2, AXES[axis]]
    length = float(np.hypot(normal[0], normal[1]))
    if length < MIN_HORIZONTAL:
        return None
    normal = normal / length
    tx, ty = float(m[0, 3]), float(m[1, 3])
    to_robot = np.array([float(robot_xy[0]) - tx, float(robot_xy[1]) - ty])
    if float(normal @ to_robot) < 0.0:
        normal = -normal
    return Pose2D(tx, ty, math.atan2(normal[1], normal[0]))


def goal_from_tag(tag: Pose2D, goal: GoalConfig) -> Pose2D:
    """Return the docked base_link pose for a tag pose (same frame)."""
    facing = math.pi if goal.direction == 'forward' else 0.0
    yaw = wrap_angle(facing + math.radians(goal.yaw_offset_deg))
    # lateral_offset is to the left of a viewer facing the tag: -y of the tag frame
    return tag @ Pose2D(goal.distance, -goal.lateral_offset, yaw)


def direction_sign(goal: GoalConfig) -> float:
    """Return +1 when the robot walks forward onto the dock, -1 when it backs in."""
    return 1.0 if goal.direction == 'forward' else -1.0


def dock_errors(robot: Pose2D, goal_pose: Pose2D, sign: float) -> DockErrors:
    """Return the errors of the robot to the docked pose (goal in the same frame)."""
    local = goal_pose.between(robot)
    return DockErrors(-sign * local.x, local.y, wrap_angle(local.yaw))


def blend(estimate: Pose2D, new: Pose2D, weight: float) -> Pose2D:
    """Return estimate moved toward new by weight (0..1), the angle on the circle."""
    w = min(max(weight, 0.0), 1.0)
    dyaw = wrap_angle(new.yaw - estimate.yaw)
    return Pose2D(
        estimate.x + w * (new.x - estimate.x),
        estimate.y + w * (new.y - estimate.y),
        wrap_angle(estimate.yaw + w * dyaw),
    )


class DockEstimate:
    """The docked pose in the odometry frame, smoothed over detections, outliers rejected."""

    def __init__(self, smoothing: float, max_jump: float, max_outliers: int):
        self.smoothing, self.max_jump, self.max_outliers = smoothing, max_jump, max_outliers
        self.goal: Pose2D | None = None
        self.frozen = False
        self.outliers = 0
        self.updates = 0

    def reset(self):
        """Forget the estimate (a new attempt starts from fresh detections)."""
        self.goal, self.frozen, self.outliers, self.updates = None, False, 0, 0

    def update(self, goal: Pose2D) -> bool:
        """Fold in the goal of one detection; return False when it was rejected or frozen."""
        if self.frozen or not goal.is_finite():
            return False
        if self.goal is None:
            self.goal, self.updates = goal, 1
            return True
        if self.goal.distance(goal) > self.max_jump:
            self.outliers += 1
            if self.outliers < self.max_outliers:
                return False
            self.goal, self.outliers, self.updates = goal, 0, 1  # the dock really moved
            return True
        self.outliers = 0
        self.goal = blend(self.goal, goal, self.smoothing)
        self.updates += 1
        return True


def field_problems(
    goal: GoalConfig, robot: RobotProfile, speed: float, margin: float, human_speed: float
) -> list[str]:
    """Return why the control mux field would stop the robot before the docked pose."""
    fp, lim = robot.footprint, robot.limits
    side, edge = ('front', fp.front) if goal.direction == 'forward' else ('rear', fp.rear)
    clearance = goal.distance - edge
    needed = margin + protective_distance(speed, lim.max_decel, lim.reaction_time, human_speed)
    if clearance >= needed:
        return []
    return [
        f'at goal.distance {goal.distance:.2f} m the {side}'
        f' edge is {clearance:.2f} m from the tag; the control field (margin {margin:.2f} m,'
        f' human_speed {human_speed:.2f} m/s) stops the robot {needed:.2f} m before an obstacle:'
        f' docking will be held short of the dock (raise goal.distance to'
        f' {edge + needed:.2f} m or lower the field values for docking)'
    ]
