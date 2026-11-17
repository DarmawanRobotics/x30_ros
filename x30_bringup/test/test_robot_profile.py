from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from x30_utils.config import build, entries, flatten
from x30_utils.robot import RobotProfile

PROFILE = Path(__file__).resolve().parents[1] / 'config' / 'robot.yaml'
# Jueying X30 API documentation, appendix B: maximum velocity of the walk gait (normal height)
WALK = {'forward': 1.2, 'backward': 0.75, 'lateral': 0.15, 'yaw': 0.45}


def profile() -> RobotProfile:
    values = flatten(yaml.safe_load(PROFILE.read_text()), 'robot.')
    assert sorted(set(values) - {e.name for e in entries(RobotProfile, 'robot.')}) == []
    return build(RobotProfile, values, 'robot.')


def test_the_profile_is_complete_and_valid():
    robot = profile()
    assert robot.name == 'x30' and robot.kinematics == 'omni' and robot.can_strafe
    assert (robot.frames.map, robot.frames.odom, robot.frames.base) == ('map', 'odom', 'base_link')


def test_the_limits_are_reachable_in_the_walk_gait():
    limits = profile().limits
    assert limits.max_vx <= min(WALK['forward'], WALK['backward'])
    assert limits.max_vy <= WALK['lateral']
    assert limits.max_wz <= WALK['yaw']


def test_the_footprint_is_the_standing_robot():
    robot = profile()
    assert robot.footprint.front + robot.footprint.rear == pytest.approx(1.0)  # 1000 mm
    assert 2.0 * robot.footprint.half_width == pytest.approx(0.47)  # 470 mm
    assert 0.3 < robot.base_height < 0.715  # below the top of the standing robot
