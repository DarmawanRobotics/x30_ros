"""Avoidance on a 360 deg front scan: only the sectors around the direction of travel count."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_navigation.core.avoidance import POLICIES, AvoidanceShared
from x30_navigation.core.avoidance.base import travel_direction
from x30_navigation.core.scan import SectorScanData
from x30_utils.robot import Footprint, Limits, RobotProfile

N = 72
INC = 2.0 * math.pi / N
NAMES = ['braitenberg', 'follow_gap', 'potential_field', 'vfh']
SHARED = AvoidanceShared(safe_distance=1.5, emergency_distance=0.75, smoothing=0.0)
SMALL = RobotProfile(
    kinematics='diff', footprint=Footprint(0.2, 0.2, 0.15), limits=Limits(max_vy=0.0)
)


def full_scan(fn) -> SectorScanData:
    """360 deg scan (angle_min -pi) from a function of the sector angle in degrees."""
    ranges = [fn(math.degrees(-math.pi + (i + 0.5) * INC)) for i in range(N)]
    return SectorScanData.of(-math.pi, INC, 8.0, ranges)


def make(name):
    return POLICIES.create(name, SHARED, robot=SMALL)


def behind_left(a: float) -> float:
    """An obstacle 0.6 m away behind and to the left of the robot (120..170 deg)."""
    return 0.6 if 120.0 < a < 170.0 else 8.0


@pytest.mark.parametrize('name', NAMES)
def test_obstacle_behind_never_steers_forward_travel(name):
    out = make(name).update(full_scan(behind_left))
    assert out.heading_offset == pytest.approx(0.0, abs=1e-9)
    assert out.lateral_bias == pytest.approx(0.0, abs=1e-9)
    assert out.velocity_scale == pytest.approx(1.0) and not out.emergency


@pytest.mark.parametrize('name', NAMES)
def test_obstacle_behind_matters_while_reversing(name):
    out = make(name).update(full_scan(lambda a: 0.6 if abs(a) > 170.0 else 8.0), math.pi)
    assert out.emergency and out.velocity_scale < 1.0


@pytest.mark.parametrize('name', NAMES)
def test_obstacle_ahead_is_ignored_while_reversing(name):
    out = make(name).update(full_scan(lambda a: 0.5 if abs(a) < 30.0 else 8.0), math.pi)
    assert not out.emergency and out.velocity_scale == pytest.approx(1.0)
    assert out.heading_offset == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('name', NAMES)
def test_wrap_around_behind_is_one_window(name):
    """Reversing: sectors at +179 and -179 deg are neighbours (the scan wraps at +-180)."""
    policy = make(name)
    angles, _ = policy.window(full_scan(lambda a: 8.0), math.pi)
    assert len(angles) == N // 2
    assert np.all(np.diff(angles) > 0.0) and abs(angles[len(angles) // 2]) < INC


@pytest.mark.parametrize('name', ['follow_gap', 'potential_field', 'vfh'])
def test_reversing_steering_is_relative_to_the_travel_direction(name):
    """Reversing, an obstacle behind-right of the robot is left of the travel: turn right."""
    out = make(name).update(full_scan(lambda a: 1.0 if -175.0 < a < -155.0 else 8.0), math.pi)
    assert -math.pi / 2 < out.heading_offset < 0.0


def test_front_scan_has_nothing_behind():
    front = SectorScanData.of(-math.pi / 2, math.pi / 32, 8.0, [8.0] * 32)
    out = make('vfh').update(front, math.pi)
    assert out.emergency and out.velocity_scale == 0.0


def test_travel_direction():
    assert travel_direction(0.3, True) == 0.0
    assert travel_direction(-0.3, True) == pytest.approx(math.pi)
    assert travel_direction(-0.3, False) == 0.0  # no rear sectors: stay forward
    assert travel_direction(-0.005, True) == 0.0
