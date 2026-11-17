"""Restart and jump detection (review finding #7)."""

from __future__ import annotations

import math

import numpy as np

from x30_odometry.core.config import JumpConfig, RestartConfig
from x30_odometry.core.restart import PoseSample, at_origin, is_jump, restart_reason

RESTART = RestartConfig()
JUMP = JumpConfig()


def pose(stamp, x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    half = math.radians(yaw_deg) / 2.0
    return PoseSample(stamp, np.array([x, y, z]), (0.0, 0.0, math.sin(half), math.cos(half)))


def test_yaw_of_pose():
    assert abs(pose(0.0, yaw_deg=30.0).yaw - math.radians(30.0)) < 1e-9


def test_turn_in_place_near_the_start_is_no_restart():
    """#7: the old check called yaw 90 -> 0 deg at 2 cm from the start a restart."""
    prev = pose(10.0, x=0.02, yaw_deg=90.0)
    for yaw in (45.0, 5.0, 0.5, 0.0):
        cur = pose(10.1, x=0.02, yaw_deg=yaw)
        assert restart_reason(prev, cur, False, RESTART) == ''
        prev = pose(10.1, x=0.02, yaw_deg=yaw)


def test_yaw_snap_alone_is_no_restart():
    prev = pose(10.0, x=0.01, yaw_deg=170.0)
    assert restart_reason(prev, pose(10.1, yaw_deg=0.0), False, RESTART) == ''


def test_gap_and_origin_is_a_restart():
    prev = pose(10.0, x=3.0, y=1.0, yaw_deg=40.0)
    reason = restart_reason(prev, pose(16.0), False, RESTART)
    assert 'origin' in reason and '6.0 s' in reason


def test_gap_at_the_origin_after_standing_there_is_a_restart():
    """A respawn while the robot stood at the start cannot be told apart: report it."""
    assert restart_reason(pose(10.0), pose(15.0), False, RESTART)


def test_gap_away_from_the_origin_is_no_restart():
    prev = pose(10.0, x=3.0)
    assert restart_reason(prev, pose(16.0, x=3.0), False, RESTART) == ''


def test_translation_jump_to_the_origin_is_a_restart():
    prev = pose(10.0, x=5.0)
    reason = restart_reason(prev, pose(10.1), True, RESTART)
    assert reason.startswith('jumped 5.00 m')


def test_short_gap_at_the_origin_without_jump_is_no_restart():
    assert restart_reason(pose(10.0, x=0.03), pose(10.5), False, RESTART) == ''


def test_time_going_back_is_a_restart():
    assert restart_reason(pose(10.0, x=3.0), pose(4.0, x=3.0), False, RESTART).startswith(
        'time went back'
    )


def test_first_pose_is_no_restart():
    assert restart_reason(None, pose(0.0), False, RESTART) == ''


def test_origin_needs_position_and_yaw():
    assert at_origin(pose(0.0, x=0.01, yaw_deg=0.5), RESTART)
    assert not at_origin(pose(0.0, x=0.2), RESTART)
    assert not at_origin(pose(0.0, yaw_deg=10.0), RESTART)
    assert not at_origin(pose(0.0, z=0.2), RESTART)


def test_jump_against_the_robot_speed_limit():
    assert is_jump(2.0, 0.1, 1.0, JUMP)  # 20 m/s for a 1 m/s robot
    assert not is_jump(0.1, 0.1, 1.0, JUMP)  # 1 m/s
    assert not is_jump(0.25, 0.01, 1.0, JUMP)  # below min_step_m (noise, bursts)
    assert not is_jump(2.0, 1.0, 1.0, JUMP)  # 2 m in 1 s: possible at 3x
