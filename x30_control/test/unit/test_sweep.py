"""The swept footprint of a straight motion: the field's translation check."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_control.core.command import Command
from x30_control.core.config import FieldConfig
from x30_control.core.field import FieldState, ProtectiveField
from x30_control.core.scan import ScanState, ScanView
from x30_control.core.sweep import box_distance, entry_distances, swept_clearance
from x30_utils.robot import Footprint, RobotProfile
from x30_utils.safety import translation_clearance

ROBOT = RobotProfile()
FP = ROBOT.footprint
HEADING = math.radians(-36.0)


def unit(angle: float) -> np.ndarray:
    return np.array([math.cos(angle), math.sin(angle)])


def leading_corner_point(angle: float, ahead: float, inward: float) -> np.ndarray:
    """A point ahead of the rotated bounding box, inside its width (on the left)."""
    u = unit(angle)
    n = np.array([-u[1], u[0]])
    corners = FP.corners()
    return ((corners @ u).max() + ahead) * u + ((corners @ n).max() - inward) * n


def test_diagonal_translation_is_not_held_by_the_rotated_bounding_box():
    """The field refused a diagonal pass where only the rotated bounding box touched."""
    point = leading_corner_point(HEADING, 0.05, 0.05)
    u = unit(HEADING)
    assert translation_clearance(FP, *u, point[None], 0.1) == pytest.approx(0.05)
    assert swept_clearance(FP, *u, point[None], 0.1) > 0.25
    field = ProtectiveField(ROBOT, FieldConfig(human_speed=0.0))
    cmd = Command(0.3 * u[0], 0.3 * u[1], 0.0)
    result = field.limit(cmd, ScanView(ScanState.OK, point[None]), remote=False)
    assert result.clearance > 0.25 and result.command.speed > 0.1


@pytest.mark.parametrize('angle_deg', [0.0, 90.0, 180.0, -90.0])
def test_axis_motion_is_the_corridor_of_x30_utils(angle_deg):
    rng = np.random.RandomState(5)
    points = rng.uniform(-2.5, 2.5, (500, 2))
    points = points[~FP.contains(points, 0.05)]
    u = np.round(unit(math.radians(angle_deg)), 12)
    for p in points:
        assert swept_clearance(FP, *u, p[None], 0.1) == pytest.approx(
            translation_clearance(FP, *u, p[None], 0.1)
        )


def test_entry_matches_a_sampled_sweep():
    """The widened footprint (+-margin across the motion) against a brute-force sweep."""
    rng = np.random.RandomState(7)
    steps = np.arange(0.0, 4.0, 0.002)
    for _ in range(60):
        u = unit(rng.uniform(-math.pi, math.pi))
        n = np.array([-u[1], u[0]])
        p = rng.uniform(-2.0, 2.0, 2)
        if box_distance(FP, p[None])[0] < 0.15:
            continue
        expected = math.inf
        for s in np.linspace(-0.1, 0.1, 11):
            moved = p[None, :] - steps[:, None] * u[None, :] - s * n[None, :]
            hit = FP.contains(moved)
            if hit.any():
                expected = min(expected, float(steps[np.argmax(hit)]))
        got = float(entry_distances(FP, u[0], u[1], p[None], 0.1)[0])
        assert (math.isinf(got) and math.isinf(expected)) or got == pytest.approx(
            expected, abs=0.005
        )


def test_point_model_beside_and_inside():
    beside = np.array([[0.0, FP.half_width + 0.05]])
    assert math.isinf(swept_clearance(FP, 1.0, 0.0, beside, 0.1))
    assert swept_clearance(FP, 0.0, 1.0, beside, 0.1) == pytest.approx(0.05)
    diagonal = unit(math.radians(45.0))
    assert swept_clearance(FP, *diagonal, beside, 0.1) == pytest.approx(
        (0.05 - 0.019) * math.sqrt(2.0)
    )
    assert swept_clearance(FP, 1.0, 0.0, np.array([[0.1, 0.0]]), 0.1) == 0.0
    assert math.isinf(swept_clearance(FP, 1.0, 0.0, np.zeros((0, 2)), 0.1))


def test_asymmetric_footprint():
    fp = Footprint(front=0.6, rear=0.2, half_width=0.3)
    assert swept_clearance(fp, -1.0, 0.0, np.array([[-1.0, 0.0]])) == pytest.approx(0.8)
    assert swept_clearance(fp, 1.0, 0.0, np.array([[1.0, 0.0]])) == pytest.approx(0.4)


def test_field_state_for_a_clear_diagonal():
    field = ProtectiveField(ROBOT, FieldConfig(human_speed=0.0))
    u = unit(HEADING)
    point = leading_corner_point(HEADING, 0.05, -0.15)  # beside the widened sweep
    cmd = Command(0.3 * u[0], 0.3 * u[1], 0.0)
    result = field.limit(cmd, ScanView(ScanState.OK, point[None]), remote=False)
    assert result.state is FieldState.CLEAR
