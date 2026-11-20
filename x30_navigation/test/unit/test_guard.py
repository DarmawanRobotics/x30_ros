"""Motion guard: the swept footprint, the lateral margin added once, the field's point model."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from rig import ROBOT, Rig, S, person
from sim import route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig, SafetyConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.guard import MotionGuard
from x30_utils.robot import Footprint
from x30_utils.safety import translation_clearance

FP = ROBOT.footprint
LM = SafetyConfig().lateral_margin
HEADING = math.radians(-36.0)  # E4 reactive 1: the VFH heading past a person on the route


def diagonal(speed: float = 0.4) -> Command:
    return Command(speed * math.cos(HEADING), speed * math.sin(HEADING))


def beside_swept_path(gap: float) -> np.ndarray:
    """A point 1.2 m along the diagonal, gap beside the swept footprint (on its left)."""
    ux, uy = math.cos(HEADING), math.sin(HEADING)
    corners = np.array([[FP.front, FP.half_width], [-FP.rear, FP.half_width]])
    reach = float((corners @ np.array([-uy, ux])).max())  # left edge of the sweep
    return np.array([[1.2 * ux - (reach + gap) * uy, 1.2 * uy + (reach + gap) * ux]])


def test_diagonal_motion_passes_where_the_body_passes():
    """E4: a diagonal course passes where the swept body passes, not the rotated box."""
    u = np.array([math.cos(HEADING), math.sin(HEADING)])
    n = np.array([-u[1], u[0]])
    corners = FP.corners()
    point = ((corners @ u).max() + 0.05) * u + ((corners @ n).max() - 0.05) * n
    assert translation_clearance(FP, *u, point[None], 0.0) == pytest.approx(0.05)
    guard = MotionGuard(ROBOT, SafetyConfig(lateral_margin=0.0))
    clearance = guard.limit(diagonal(), point[None], False).clearance
    steps = np.arange(0.0, 2.0, 0.001)
    touched = FP.contains(point[None, :] - steps[:, None] * u[None, :])
    assert clearance == pytest.approx(steps[np.argmax(touched)], abs=2e-3)
    assert clearance > 0.25


def test_lateral_margin_is_added_once():
    guard = MotionGuard(ROBOT, SafetyConfig())
    assert math.isinf(guard.limit(diagonal(), beside_swept_path(LM + 0.005), False).clearance)
    inside = guard.limit(diagonal(), beside_swept_path(LM - 0.005), False)
    assert math.isfinite(inside.clearance) and inside.clearance < 1.2


@pytest.mark.parametrize('direction', [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)])
def test_axis_motion_keeps_the_corridor_of_the_field(direction):
    """Along a body axis the sweep is the corridor of x30_utils.safety (no change there)."""
    rng = np.random.RandomState(3)
    points = rng.uniform(-2.5, 2.5, (400, 2))
    points = points[~FP.contains(points, 0.05)]
    guard = MotionGuard(ROBOT, SafetyConfig())
    for p in points:
        cmd = Command(0.3 * direction[0], 0.3 * direction[1])
        expected = translation_clearance(FP, *direction, p[None], LM)
        assert guard.limit(cmd, p[None], False).clearance == pytest.approx(expected)


def test_point_inside_the_body_or_blind_stops():
    guard = MotionGuard(ROBOT, SafetyConfig())
    assert guard.limit(diagonal(), np.array([[0.1, 0.1]]), False).stalled
    assert guard.limit(diagonal(), np.zeros((0, 2)), True).stalled


def test_point_beside_the_body_may_be_passed_not_approached():
    """The field's arc check rule: a point in the margin band beside the body is passed."""
    guard = MotionGuard(ROBOT, SafetyConfig())
    beside = np.array([[0.0, FP.half_width + 0.05]])
    assert math.isinf(guard.limit(Command(0.3), beside, False).clearance)
    assert guard.limit(Command(0.0, 0.2), beside, False).clearance < 0.05


def test_person_beside_the_route_is_passed_on_the_field_point_model():
    """E4: a person 0.15 m beside is passed on the field's point model (sector centres)."""
    config = FollowerConfig(require_localization=False, avoidance_mode='off')
    rig = Rig(route_of(straight_route(0.0, 8.0)), config)
    rig.sectors = 72
    rig.obstacles = person(4.0, -(FP.half_width + 0.15 + 0.25), n=180)
    assert rig.follower.start(rig.pose)[0]
    rig.run(25.0, until=lambda d: rig.pose.x > 5.0)
    assert rig.pose.x > 5.0
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_sweep_is_the_same_in_both_packages():
    """core/sweep.py is shared with x30_control (until it moves to x30_utils.safety)."""
    package = Path(__file__).resolve().parents[2]
    ours = package / 'src' / 'x30_navigation' / 'core' / 'sweep.py'
    theirs = package.parent / 'x30_control' / 'src' / 'x30_control' / 'core' / 'sweep.py'
    assert ours.is_file(), ours
    if not theirs.is_file():  # a workspace with this package alone
        pytest.skip('x30_control is not next to this package')
    assert ours.read_text() == theirs.read_text()


def test_asymmetric_footprint_sweeps_its_own_corners():
    fp = Footprint(front=0.6, rear=0.2, half_width=0.3)
    robot = type(ROBOT)(footprint=fp, limits=ROBOT.limits)
    behind = np.array([[-1.0, 0.0]])
    clearance = MotionGuard(robot, SafetyConfig()).limit(Command(-0.2), behind, False).clearance
    assert clearance == pytest.approx(0.8)


def test_a_point_passed_in_the_margin_band_is_held_like_the_field_holds_it():
    """E4: a point passed in the margin band is held margin short, like the field."""
    safety = SafetyConfig()
    guard = MotionGuard(ROBOT, safety)
    in_band = np.array([[FP.front + 0.4, -(FP.half_width + LM - 0.02)]])
    ahead = np.array([[FP.front + 0.4, 0.0]])
    passing = guard.limit(Command(0.3), in_band, False)
    hitting = guard.limit(Command(0.3), ahead, False)
    assert passing.clearance == pytest.approx(0.4) == hitting.clearance
    assert passing.command.vx == pytest.approx(guard.allowed_speed(0.4, safety.pass_margin))
    assert hitting.command.vx == pytest.approx(guard.allowed_speed(0.4))
    assert passing.command.vx > hitting.command.vx + 0.05
    assert SafetyConfig(pass_margin=0.1).check_control(LM, 0.5, 0.15)
    assert not safety.check_control(LM, safety.human_speed, 0.15)


def test_a_slowing_ramp_never_outruns_the_guard():
    guard = MotionGuard(ROBOT, SafetyConfig())
    ahead = np.array([[FP.front + 0.6, 0.0]])
    allowed = guard.limit(Command(0.4), ahead, False).command
    assert allowed.vx < 0.35  # the guard slows the target
    published = guard.published(Command(0.38), allowed, ahead, False)  # the ramp still at 0.38
    assert published.vx == pytest.approx(allowed.vx)


def test_a_ramp_bent_toward_something_goes_the_guarded_way():
    """From a standstill the smoothed sideways speed bends the ramp: the guarded way instead."""
    guard = MotionGuard(ROBOT, SafetyConfig())
    right_front = np.array([[0.5, -0.5]])
    guarded = guard.limit(Command(0.4, -0.024), right_front, False).command
    assert math.hypot(guarded.vx, guarded.vy) > 0.39
    published = guard.published(Command(0.025, -0.012), guarded, right_front, False)
    assert math.hypot(published.vx, published.vy) == pytest.approx(math.hypot(0.025, 0.012))
    assert math.atan2(published.vy, published.vx) == pytest.approx(math.atan2(-0.024, 0.4))
