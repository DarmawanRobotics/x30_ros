"""Dock geometry: tag normal sign, docked pose, errors, the smoothed estimate, the control law."""

from __future__ import annotations

import math

from dock_rig import TAG, tag_matrix
import numpy as np

from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.docking.control import DockController, DockLimits
from x30_navigation.core.docking.geometry import (
    DockErrors,
    DockEstimate,
    dock_errors,
    goal_from_tag,
    tag_pose,
)
from x30_utils.math import Pose2D

CFG = DockingConfig()


def test_tag_normal_points_at_the_robot_whatever_the_detector_convention():
    for z_toward_viewer in (True, False):
        pose = tag_pose(tag_matrix(TAG, z_toward_viewer), (0.0, 0.0))
        assert pose is not None
        assert math.isclose(pose.x, TAG.x) and math.isclose(pose.y, TAG.y)
        assert abs(math.remainder(pose.yaw - math.pi, 2 * math.pi)) < 1e-9


def test_tag_on_the_floor_or_invalid_is_refused():
    floor = np.eye(4)  # z up: the plate lies flat
    assert tag_pose(floor, (0.0, 0.0)) is None
    assert tag_pose(np.full((4, 4), np.nan), (0.0, 0.0)) is None
    assert tag_pose(tag_matrix(TAG), (0.0, 0.0), axis='x') is not None  # in the plate plane


def test_goal_in_front_of_the_tag_forward_and_backward():
    forward = goal_from_tag(TAG, CFG.goal)
    assert math.isclose(forward.x, TAG.x - CFG.goal.distance) and abs(forward.y) < 1e-12
    assert abs(math.remainder(forward.yaw, 2 * math.pi)) < 1e-12  # facing the tag (+x)
    backward = goal_from_tag(TAG, CFG.goal.__class__(distance=0.5, direction='backward'))
    assert abs(math.remainder(backward.yaw - math.pi, 2 * math.pi)) < 1e-12
    left = goal_from_tag(TAG, CFG.goal.__class__(distance=0.5, lateral_offset=0.2))
    assert math.isclose(left.y, 0.2)  # left of a viewer facing the tag


def test_errors_in_the_goal_frame():
    goal = Pose2D(2.0, 0.0, 0.0)
    e = dock_errors(Pose2D(1.0, 0.1, 0.2), goal, 1.0)
    assert math.isclose(e.remaining, 1.0) and math.isclose(e.lateral, 0.1)
    assert math.isclose(e.yaw, 0.2)
    back = dock_errors(Pose2D(3.0, 0.0, 0.0), goal, -1.0)
    assert math.isclose(back.remaining, 1.0)


def test_estimate_smooths_rejects_outliers_and_restarts():
    est = DockEstimate(smoothing=0.5, max_jump=0.2, max_outliers=3)
    assert est.update(Pose2D(1.0, 0.0, 0.0))
    assert est.update(Pose2D(1.1, 0.0, 0.0)) and math.isclose(est.goal.x, 1.05)
    assert not est.update(Pose2D(3.0, 0.0, 0.0))  # outlier
    assert not est.update(Pose2D(3.0, 0.0, 0.0))
    assert est.update(Pose2D(3.0, 0.0, 0.0)) and est.goal.x == 3.0  # the dock moved
    est.frozen = True
    assert not est.update(Pose2D(3.05, 0.0, 0.0)) and est.goal.x == 3.0
    angles = DockEstimate(0.5, 1.0, 3)
    angles.update(Pose2D(0.0, 0.0, math.pi - 0.1))
    angles.update(Pose2D(0.0, 0.0, -math.pi + 0.1))  # across the wrap
    assert abs(abs(angles.goal.yaw) - math.pi) < 1e-9


def test_omni_centring_command_slides_and_turns_without_walking():
    ctl = DockController(CFG.centering, CFG.approach, True, DockLimits())
    cmd = ctl.center(DockErrors(remaining=1.0, lateral=0.2, yaw=0.0))
    assert abs(cmd.vx) < 1e-9 and cmd.vy < 0.0  # slide right, back onto the line
    turn = ctl.center(DockErrors(remaining=1.0, lateral=0.0, yaw=0.3))
    assert turn.wz < 0.0
    assert not ctl.centered(DockErrors(1.0, 0.2, 0.0)) and ctl.centered(DockErrors(1.0, 0.0, 0.0))
    walk = ctl.walk(DockErrors(remaining=1.0, lateral=0.0, yaw=0.0))
    assert math.isclose(walk.vx, CFG.approach.speed)
    assert ctl.walk(DockErrors(remaining=-0.01, lateral=0.0, yaw=0.0)).vx == 0.0
    assert ctl.off_line(DockErrors(1.0, 0.2, 0.0))
    assert not ctl.off_line(DockErrors(0.1, 0.2, 0.0))  # inside blind_distance: go on


def test_diff_centring_points_at_the_line_and_limits_hold():
    ctl = DockController(CFG.centering, CFG.approach, False, DockLimits(0.05, 0.0, 0.2))
    cmd = ctl.center(DockErrors(remaining=1.0, lateral=0.2, yaw=0.0))
    assert cmd.vx == 0.0 and cmd.vy == 0.0 and cmd.wz < 0.0  # turn right toward the line
    assert abs(cmd.wz) <= 0.2
    walk = ctl.walk(DockErrors(remaining=1.0, lateral=0.0, yaw=0.0))
    assert walk.vy == 0.0 and math.isclose(walk.vx, 0.05)  # capped by the robot


def test_field_check_warns_when_the_goal_is_inside_the_mux_field():
    from x30_navigation.core.docking.geometry import field_problems
    from x30_utils.robot import RobotProfile

    robot = RobotProfile()  # front 0.40 m
    close = CFG.goal.__class__(distance=0.55)
    problems = field_problems(close, robot, 0.03, margin=0.15, human_speed=0.5)
    assert problems and 'goal.distance' in problems[0]
    assert not field_problems(close, robot, 0.03, margin=0.0, human_speed=0.0)
    far = CFG.goal.__class__(distance=0.8)
    assert not field_problems(far, robot, 0.03, margin=0.15, human_speed=0.5)
