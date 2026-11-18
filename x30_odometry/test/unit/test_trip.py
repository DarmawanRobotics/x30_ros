"""TripMeter: distance by steps, speed over a window, moving time, gaps, reset, kept totals."""

from __future__ import annotations

import math

import pytest

from x30_odometry.core.config import TripConfig
from x30_odometry.core.trip import TripMeter, TripTotals
from x30_utils.math import default_rng

HZ = 10.0


def drive(meter: TripMeter, t0: float, x0: float, speed: float, seconds: float, y: float = 0.0):
    """Feed poses at 10 Hz along x at a constant speed; return the end time and x."""
    n = round(seconds * HZ)
    t, x = t0, x0
    for _ in range(n):
        t += 1.0 / HZ
        x += speed / HZ
        meter.add(t, x, y)
    return t, x


def test_a_straight_run_counts_its_distance_speed_and_time():
    meter = TripMeter(TripConfig())
    meter.add(100.0, 0.0, 0.0)
    drive(meter, 100.0, 0.0, 0.5, 10.0)
    assert meter.distance == pytest.approx(5.0, abs=0.06)  # whole 5 cm steps
    assert meter.speed == pytest.approx(0.5, abs=0.01)
    assert meter.max_speed == pytest.approx(0.5, abs=0.01)
    assert meter.average == pytest.approx(0.5, abs=0.02)
    assert meter.moving_sec == pytest.approx(10.0, abs=0.15)


def test_a_standing_robot_adds_nothing():
    rng = default_rng(7)
    meter = TripMeter(TripConfig())
    for k in range(600):  # a minute of 1 cm pose noise
        meter.add(100.0 + k / HZ, *rng.normal(0.0, 0.01, 2))
    assert meter.distance == 0.0
    assert meter.speed == 0.0
    assert meter.moving_sec == 0.0
    assert meter.max_speed == 0.0


def test_stop_and_go_counts_only_the_moving_time():
    meter = TripMeter(TripConfig())
    meter.add(0.0, 0.0, 0.0)
    t, x = drive(meter, 0.0, 0.0, 0.4, 5.0)
    t, x = drive(meter, t, x, 0.0, 20.0)  # waiting for a person
    assert meter.speed == 0.0
    drive(meter, t, x, 0.4, 5.0)
    assert meter.distance == pytest.approx(4.0, abs=0.1)
    assert meter.moving_sec == pytest.approx(10.0, abs=0.3)
    assert meter.average == pytest.approx(0.4, abs=0.02)


def test_nothing_is_counted_across_a_gap_a_jump_or_a_restart():
    meter = TripMeter(TripConfig())
    meter.add(0.0, 0.0, 0.0)
    t, x = drive(meter, 0.0, 0.0, 0.5, 2.0)
    before = meter.distance
    meter.add(t + 5.0, x + 3.0, 0.0)  # 5 s of silence, then 3 m further: not travelled
    assert meter.distance == before and meter.speed == 0.0
    t, x = t + 5.0, x + 3.0
    meter.add(t + 0.1, x + 2.0, 0.0)  # 20 m/s: a jump of the estimator
    assert meter.distance == before
    t, x = drive(meter, t + 0.1, x + 2.0, 0.5, 2.0)
    assert meter.distance == pytest.approx(before + 1.0, abs=0.1)
    meter.gap()  # the monitor saw a restart: the next pose is a new origin
    meter.add(t + 0.1, 0.0, 0.0)
    assert meter.distance == pytest.approx(before + 1.0, abs=0.1)
    assert meter.speed == 0.0


def test_a_stamp_going_back_is_a_new_stretch():
    meter = TripMeter(TripConfig())
    meter.add(10.0, 0.0, 0.0)
    meter.add(10.1, 0.04, 0.0)
    meter.add(9.0, 1.0, 0.0)
    meter.add(9.1, 1.04, 0.0)
    assert meter.distance == 0.0


def test_planar_distance_and_curves():
    meter = TripMeter(TripConfig())
    radius, n = 2.0, 400  # a full circle in 40 s
    for k in range(n + 1):
        a = 2.0 * math.pi * k / n
        meter.add(k / HZ, radius * math.cos(a), radius * math.sin(a))
    assert meter.distance == pytest.approx(2.0 * math.pi * radius, rel=0.01)


def test_reset_starts_from_zero_and_keeps_the_stretch():
    meter = TripMeter(TripConfig())
    meter.add(0.0, 0.0, 0.0)
    t, x = drive(meter, 0.0, 0.0, 0.5, 4.0)
    meter.reset(since=1_700_000_000.0)
    assert (meter.distance, meter.moving_sec, meter.max_speed) == (0.0, 0.0, 0.0)
    assert meter.since == 1_700_000_000.0
    drive(meter, t, x, 0.5, 2.0)  # no jump from the old anchor
    assert meter.distance == pytest.approx(1.0, abs=0.06)


def test_totals_round_trip_and_bad_files_are_refused():
    meter = TripMeter(TripConfig(), since=1_700_000_000.0)
    meter.add(0.0, 0.0, 0.0)
    drive(meter, 0.0, 0.0, 0.5, 4.0)
    assert meter.changed
    kept = meter.totals()
    assert not meter.changed
    again = TripMeter(TripConfig())
    again.restore(TripTotals.from_dict(kept.to_dict()))
    assert again.distance == pytest.approx(kept.distance, abs=1e-3)
    assert again.moving_sec == pytest.approx(kept.moving_sec, abs=0.1)
    assert again.max_speed == pytest.approx(kept.max_speed, abs=1e-3)
    assert again.since == 1_700_000_000.0
    for bad in (None, [], {}, {'distance_m': 'x'}, {**kept.to_dict(), 'moving_sec': -1.0},
                {**kept.to_dict(), 'since': float('nan')}):  # fmt: skip
        assert TripTotals.from_dict(bad) is None


def test_live_configuration_applies():
    meter = TripMeter(TripConfig())
    meter.configure(TripConfig(moving_mps=1.0))
    meter.add(0.0, 0.0, 0.0)
    drive(meter, 0.0, 0.0, 0.5, 3.0)
    assert meter.speed == 0.0 and meter.moving_sec == 0.0
    assert meter.distance == pytest.approx(1.5, abs=0.06)  # distance does not depend on it
