from __future__ import annotations

import math

import pytest

from x30_control.core.command import ZERO, Command, speed_ratio
from x30_control.core.envelope import Envelope, HealthLimit
from x30_utils.config import build
from x30_utils.robot import RobotProfile

NAN, INF = float('nan'), float('inf')


def robot(**values) -> RobotProfile:
    return build(RobotProfile, {f'robot.{k}': v for k, v in values.items()}, 'robot.')


OMNI = Envelope(robot(**{'limits.max_vx': 1.0, 'limits.max_vy': 0.5, 'limits.max_wz': 1.5}))


def test_inside_the_envelope_is_untouched():
    result = OMNI.apply(Command(0.4, 0.1, 0.5))
    assert result.command == Command(0.4, 0.1, 0.5)
    assert result.finite and not result.clipped


def test_linear_scaled_together_keeps_the_heading():
    result = OMNI.apply(Command(2.0, 1.0, -3.0))
    assert result.command == Command(1.0, 0.5, -1.5) and result.clipped
    out = OMNI.apply(Command(0.5, -1.0, 0.0)).command
    assert (out.vx, out.vy) == (0.25, -0.5)
    assert math.atan2(out.vy, out.vx) == pytest.approx(math.atan2(-1.0, 0.5))


@pytest.mark.parametrize('bad', [NAN, INF, -INF])
def test_non_finite_on_any_axis_is_zero(bad):
    """Finding #5: NaN never slips through the envelope."""
    for cmd in (Command(bad, 0.0, 0.0), Command(0.0, bad, 0.0), Command(0.1, 0.0, bad)):
        result = OMNI.apply(cmd)
        assert result.command == ZERO and not result.finite


def test_diff_kinematics_drops_vy_but_keeps_vx():
    diff = Envelope(robot(kinematics='diff', **{'limits.max_vy': 0.0}))
    out = diff.apply(Command(0.3, 0.2, 0.4)).command
    assert out == Command(0.3, 0.0, 0.4)


def test_omni_with_no_sideways_speed_stops_a_diagonal():
    """An omni robot limited to max_vy = 0 must not turn a diagonal into a straight line."""
    env = Envelope(robot(**{'limits.max_vy': 0.0}))
    assert env.apply(Command(0.3, 0.2, 0.0)).command.speed == 0.0
    assert env.apply(Command(0.3, 0.0, 0.0)).command == Command(0.3, 0.0, 0.0)


def test_health_limit_scales_only_while_fresh():
    health = HealthLimit(enabled=True, timeout_sec=3.0)
    assert health.scale(0.0) == 1.0  # nothing received yet
    health.update(0.5, now=10.0)
    assert health.scale(12.0) == 0.5
    assert health.scale(13.5) == 1.0  # stale health: no change
    health.update(0.0, now=20.0)
    assert health.scale(20.1) == 0.0


@pytest.mark.parametrize(
    ('requested', 'applied'), [(NAN, 0.0), (INF, 0.0), (-1.0, 0.0), (7.0, 1.0)]
)
def test_health_limit_invalid_values_fail_safe(requested, applied):
    health = HealthLimit(enabled=True, timeout_sec=3.0)
    health.update(requested, now=1.0)
    assert health.scale(1.0) == applied


def test_health_limit_disabled_never_scales():
    health = HealthLimit(enabled=False, timeout_sec=3.0)
    health.update(0.0, now=1.0)
    assert health.scale(1.0) == 1.0


def test_command_helpers():
    assert Command(0.005, -0.005, 0.009).is_neutral(0.01)
    assert not Command(0.0, 0.0, 0.02).is_neutral(0.01)
    assert not Command(NAN, 0.0, 0.0).is_neutral(0.01)
    assert Command(0.3, 0.4, 0.2).scaled(0.5) == Command(0.15, 0.2, 0.1)
    assert speed_ratio(Command(1.0, 0.0, 1.0), Command(0.5, 0.0, 0.2)) == pytest.approx(0.2)
    assert speed_ratio(ZERO, ZERO) == 1.0
    assert speed_ratio(Command(1.0, 0.0, 0.0), ZERO) == 0.0
