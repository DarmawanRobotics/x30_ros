"""Latency: sensor stamps t - L, delayed commands with timeout, pose history lookups."""

from __future__ import annotations

import math

import pytest

from x30_sim.core.body import Twist
from x30_sim.core.latency import CommandQueue, PoseHistory, sensor_stamp
from x30_utils.math import Pose2D


def test_sensor_stamp_is_now_minus_latency():
    """S1: a measurement published at t carries the stamp t - L."""
    assert sensor_stamp(100.0, 0.05) == pytest.approx(99.95)
    assert sensor_stamp(100.0, 0.0) == 100.0


def test_commands_act_after_the_latency():
    """A command received at t acts from t + L; before that the previous command acts."""
    queue = CommandQueue(0.1, 0.5, Twist())
    queue.push(1.0, Twist(0.3))
    assert queue.active(1.05) == Twist()
    assert queue.active(1.1) == Twist(0.3)
    queue.push(1.2, Twist(0.6))
    assert queue.active(1.25) == Twist(0.3)
    assert queue.active(1.31) == Twist(0.6)


def test_stale_commands_become_zero():
    """A command older than the timeout (after the latency) is zero, like the driver's watchdog."""
    queue = CommandQueue(0.1, 0.5, Twist())
    queue.push(0.0, Twist(1.0))
    assert queue.active(0.55) == Twist(1.0)
    assert queue.active(0.65) == Twist()


def test_clock_jump_back_clears_the_queue():
    """Commands from a time ahead of a jumped-back clock are dropped."""
    queue = CommandQueue(0.0, 0.5, Twist())
    queue.push(10.0, Twist(1.0))
    queue.push(2.0, Twist(0.2))
    assert queue.active(2.0) == Twist(0.2)


def test_pose_history_interpolates_and_clamps():
    """Lookups between samples interpolate (yaw across +-pi too), outside they clamp."""
    history = PoseHistory(horizon_sec=1.0)
    assert history.at(0.0) is None
    history.add(0.0, Pose2D(0.0, 0.0, math.pi - 0.1))
    history.add(1.0, Pose2D(1.0, 2.0, -math.pi + 0.1))
    mid = history.at(0.5)
    assert (mid.x, mid.y) == pytest.approx((0.5, 1.0))
    assert abs(abs(mid.yaw) - math.pi) < 1e-9
    assert history.at(-1.0) == Pose2D(0.0, 0.0, math.pi - 0.1)
    assert history.at(5.0).x == 1.0


def test_pose_history_forgets_beyond_the_horizon():
    """Samples older than the horizon are dropped; lookups that old clamp to the oldest kept."""
    history = PoseHistory(horizon_sec=0.5)
    for i in range(20):
        history.add(i * 0.1, Pose2D(float(i), 0.0, 0.0))
    assert history.at(0.0).x >= 13.0
    assert history.at(1.85).x == pytest.approx(18.5)
