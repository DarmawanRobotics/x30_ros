"""Stillness and tilt maths, and the wait_still gate decision (finding O1)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_odometry.core.config import StillConfig
from x30_odometry.core.still import StillGate, StillWindow, angle_to_up, measure
from x30_utils.math import default_rng


def samples(n=200, tilt_deg=0.0, noise=0.0, gyro=0.0, scale=1.0):
    """Return (gyro, acc) of an IMU tilted about y, accelerometer in g (or scaled)."""
    rng = default_rng(0)
    t = math.radians(tilt_deg)
    acc = np.tile([math.sin(t), 0.0, math.cos(t)], (n, 1)) + rng.normal(0.0, noise, (n, 3))
    return np.full((n, 3), gyro), acc * scale


def feed(gate_or_window, seconds, rate_hz=100.0, t0=0.0, tilt_deg=0.0, gyro=0.0):
    """Feed constant IMU samples; return the time of the last one."""
    t = t0
    acc = (math.sin(math.radians(tilt_deg)), 0.0, math.cos(math.radians(tilt_deg)))
    for k in range(int(seconds * rate_hz)):
        t = t0 + k / rate_hz
        gate_or_window.add(t, (0.0, 0.0, gyro), acc)
    return t


def test_still_level_imu_passes():
    s = measure(*samples(noise=0.002))
    assert s.gyro_max < 0.03 and s.acc_rel_std < 0.01 and math.degrees(s.tilt) < 1.0
    assert not s.problems(0.03, 0.01, math.radians(3.0))


def test_tilt_and_rotation_are_reported():
    s = measure(*samples(tilt_deg=15.0))
    assert math.degrees(s.tilt) == pytest.approx(15.0, abs=0.01)
    assert s.problems(0.03, 0.01, math.radians(3.0)) == ['tilted 15.0 deg']
    assert not s.problems(0.03, 0.01, 0.0)  # tilt check off
    assert measure(*samples(gyro=0.1)).problems(0.03, 0.01)[0].startswith('rotating')


def test_shaking_is_unit_free():
    in_g = measure(*samples(noise=0.05)).acc_rel_std
    in_si = measure(*samples(noise=0.05, scale=9.81)).acc_rel_std
    assert in_g > 0.01 and in_g == pytest.approx(in_si, rel=1e-9)


def test_zero_acceleration_is_not_still():
    s = measure(np.zeros((10, 3)), np.zeros((10, 3)))
    assert math.isinf(s.acc_rel_std) and s.tilt == pytest.approx(math.pi)


def test_angle_to_up():
    assert angle_to_up((0.0, 0.0, 2.0)) == pytest.approx(0.0)
    assert angle_to_up((1.0, 0.0, 0.0)) == pytest.approx(math.pi / 2)
    assert angle_to_up((0.0, 0.0, 0.0)) == pytest.approx(math.pi)


def test_window_needs_full_span_and_fresh_samples():
    window = StillWindow(1.0, min_samples=20)
    t = feed(window, 0.5)
    assert window.measure(t) is None  # half a window
    t = feed(window, 1.2, t0=t + 0.01)
    assert window.measure(t) is not None
    assert window.measure(t + 1.0) is None  # IMU went silent


def test_window_drops_non_finite_and_time_going_back():
    window = StillWindow(1.0, min_samples=2)
    window.add(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    window.add(0.1, (math.nan, 0.0, 0.0), (0.0, 0.0, 1.0))
    assert len(window.samples) == 1
    window.add(-5.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    assert len(window.samples) == 1 and window.samples[0][0] == -5.0


def test_default_start_tilt_limit_is_three_degrees():
    """O1: the estimator may start at most 3 deg tilted (was 8)."""
    assert StillConfig().max_tilt_deg == 3.0


@pytest.mark.parametrize(('tilt_deg', 'released'), [(2.0, True), (5.0, False), (8.0, False)])
def test_gate_holds_a_tilted_start(tilt_deg, released):
    gate = StillGate(StillConfig(still_sec=1.0, timeout_sec=0.0), start=0.0)
    t = feed(gate, 1.5, tilt_deg=tilt_deg)
    decision = gate.decide(t)
    assert decision.release is released and not decision.forced
    if not released:
        assert decision.reason.startswith('tilted')


def test_gate_holds_while_turning_then_releases_when_still():
    gate = StillGate(StillConfig(still_sec=1.0, timeout_sec=0.0), start=0.0)
    t = feed(gate, 1.5, gyro=0.2)
    assert not gate.decide(t).release
    t = feed(gate, 1.2, t0=t + 0.01)
    assert gate.decide(t).release


def test_gate_waits_for_data_then_times_out():
    gate = StillGate(StillConfig(still_sec=1.0, timeout_sec=10.0), start=0.0)
    decision = gate.decide(5.0)
    assert not decision.release and decision.reason == 'no IMU data yet'
    decision = gate.decide(10.5)
    assert decision.release and decision.forced


def test_gate_without_timeout_waits_forever():
    gate = StillGate(StillConfig(still_sec=1.0, timeout_sec=0.0), start=0.0)
    feed(gate, 1.5, tilt_deg=20.0)
    assert not gate.decide(1e6).release
