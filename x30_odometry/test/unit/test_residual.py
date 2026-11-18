"""Speed residual chi^2 + CUSUM against the robot odometry (finding O1)."""

from __future__ import annotations

import math

import pytest

from x30_odometry.core.config import SpeedConfig
from x30_odometry.core.residual import SpeedResidual
from x30_utils.math import default_rng


def drive(residual, seconds, lio_mps, leg_mps, t0=0.0, x0=0.0, rate_hz=10.0, noise=0.0, seed=0):
    """Move the estimator at lio_mps and report leg_mps; return the alarm history and t, x."""
    rng = default_rng(seed)
    alarms, t, x = [], t0, x0
    for _ in range(int(seconds * rate_hz)):
        t += 1.0 / rate_hz
        x += lio_mps / rate_hz
        residual.add_leg(t, leg_mps + rng.normal(0.0, noise), 0.0)
        alarms.append(residual.add_pose(t, t, x + rng.normal(0.0, noise * 0.05), 0.0))
    return alarms, t, x


def test_agreeing_speeds_never_alarm_with_sensor_noise():
    residual = SpeedResidual(SpeedConfig())
    alarms, _, _ = drive(residual, 120.0, 0.6, 0.6, noise=0.1)
    assert not any(alarms)


def test_standing_still_is_consistent():
    residual = SpeedResidual(SpeedConfig())
    alarms, _, _ = drive(residual, 10.0, 0.0, 0.0)
    assert not any(alarms) and residual.speed == 0.0


def test_lasting_mismatch_alarms_after_a_while_and_recovers():
    """A degenerate corridor: the estimator stops while the legs walk at 0.5 m/s."""
    residual = SpeedResidual(SpeedConfig())
    drive(residual, 2.0, 0.5, 0.5)
    alarms, t, x = drive(residual, 4.0, 0.0, 0.5, t0=2.0, x0=1.0)
    first = alarms.index(True)
    assert 2 <= first <= 20, first  # neither on one sample nor never
    assert residual.alarm and residual.chi2 > SpeedConfig().cusum_drift
    alarms, _, _ = drive(residual, 3.0, 0.5, 0.5, t0=t, x0=x)
    assert not alarms[-1]


def test_threshold_scales_with_the_variances():
    """The same 0.3 m/s mismatch alarms with precise sensors, not with noisy ones."""
    precise = SpeedResidual(SpeedConfig(sigma_lio_mps=0.02, sigma_leg_mps=0.05))
    noisy = SpeedResidual(SpeedConfig(sigma_lio_mps=0.1, sigma_leg_mps=0.3))
    assert any(drive(precise, 5.0, 0.8, 0.5)[0])
    assert not any(drive(noisy, 5.0, 0.8, 0.5)[0])


def test_chi2_is_the_normalized_squared_residual():
    config = SpeedConfig(sigma_lio_mps=0.1, sigma_leg_mps=0.2)
    residual = SpeedResidual(config)
    drive(residual, 1.0, 1.0, 0.5)
    assert residual.speed == pytest.approx(1.0)
    assert residual.chi2 == pytest.approx(0.25 / (0.01 + 0.04))


def test_stale_leg_odometry_clears_the_test():
    residual = SpeedResidual(SpeedConfig())
    drive(residual, 4.0, 0.0, 0.8)
    assert residual.alarm
    residual.add_pose(10.0, 10.0, 0.0, 0.0)  # legs silent since t = 4 s
    assert not residual.alarm and residual.cusum.value == 0.0


def test_reset_and_reconfigure():
    residual = SpeedResidual(SpeedConfig())
    drive(residual, 4.0, 0.0, 0.8)
    residual.configure(SpeedConfig(cusum_threshold=1000.0))
    assert residual.cusum.threshold == 1000.0
    residual.reset()
    assert not residual.alarm and not residual.poses and residual.speed == 0.0
    assert math.isfinite(residual.leg_speed)


def test_cusum_is_bounded_for_a_quick_recovery():
    residual = SpeedResidual(SpeedConfig())
    drive(residual, 60.0, 0.0, 1.0)
    assert residual.cusum.value <= 2.0 * SpeedConfig().cusum_threshold
