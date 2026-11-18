"""ISO/TS 15066 speed and separation: the protective distance includes a walking person."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_control.core.command import Command
from x30_control.core.config import FieldConfig
from x30_control.core.field import FieldState, ProtectiveField
from x30_control.core.scan import ScanState, ScanView
from x30_control.core.separation import protective_distance, separation_speed
from x30_utils.robot import RobotProfile
from x30_utils.safety import stopping_speed

ROBOT = RobotProfile()
LIM = ROBOT.limits


def view(points) -> ScanView:
    return ScanView(ScanState.OK, np.asarray(points, dtype=float).reshape(-1, 2))


@pytest.mark.parametrize('room', [0.05, 0.3, 1.0, 2.5])
@pytest.mark.parametrize('human', [0.0, 0.2, 0.5, 1.6])
def test_separation_speed_inverts_the_protective_distance(room, human):
    v = separation_speed(room, LIM.max_decel, LIM.reaction_time, human)
    if v > 0.0:
        d = protective_distance(v, LIM.max_decel, LIM.reaction_time, human)
        assert d == pytest.approx(room)
    else:
        assert room <= human * LIM.reaction_time + 1e-12


def test_without_a_person_it_is_the_stopping_model():
    for room in (0.0, 0.2, 1.3, math.inf):
        assert separation_speed(room, 0.6, 0.25, 0.0) == pytest.approx(
            stopping_speed(room, 0.6, 0.25)
        )


def test_a_walking_person_keeps_the_robot_further_away():
    """E4 beside_route 15: v_h = 0.2 m/s brushed the robot with the static model (v_h = 0)."""
    person = [(0.9, 0.0)]
    static = ProtectiveField(ROBOT, FieldConfig(human_speed=0.0))
    moving = ProtectiveField(ROBOT, FieldConfig(human_speed=0.2))
    cmd = Command(0.6, 0.0, 0.0)
    fast = static.limit(cmd, view(person), remote=False).command.vx
    slow = moving.limit(cmd, view(person), remote=False).command.vx
    assert 0.0 < slow < fast
    room = 0.9 - ROBOT.footprint.front - FieldConfig().margin
    assert protective_distance(slow, LIM.max_decel, LIM.reaction_time, 0.2) == pytest.approx(room)


def test_default_human_speed_is_not_zero():
    """The safer default: a person is assumed to walk toward the robot."""
    assert FieldConfig().human_speed > 0.2


def test_a_standing_robot_is_not_limited():
    field = ProtectiveField(ROBOT, FieldConfig())
    result = field.limit(Command(), view([(0.5, 0.0)]), remote=False)
    assert result.state is FieldState.CLEAR


def test_close_person_stops_a_slow_approach():
    field = ProtectiveField(ROBOT, FieldConfig(human_speed=0.5))
    edge = ROBOT.footprint.front + FieldConfig().margin
    near = [(edge + 0.5 * LIM.reaction_time - 0.01, 0.0)]
    assert field.limit(Command(0.1), view(near), remote=False).state is FieldState.STOP


def test_combined_motion_keeps_the_person_distance():
    static = ProtectiveField(ROBOT, FieldConfig(human_speed=0.0))
    moving = ProtectiveField(ROBOT, FieldConfig(human_speed=0.5))
    cmd = Command(0.4, 0.0, 0.5)
    points = view([(0.9, 0.1)])
    slow = moving.limit(cmd, points, False).command
    assert slow.vx < static.limit(cmd, points, False).command.vx
