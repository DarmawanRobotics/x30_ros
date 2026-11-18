"""Body IMU: gravity at rest, linear, centripetal and angular acceleration at its origin."""

from __future__ import annotations

import numpy as np
import pytest

from x30_sim.core.body import Body, BodyLimits, Twist
from x30_sim.core.imu import GRAVITY, imu_sample
from x30_utils.math import Pose2D, default_rng

IMU_XY = (0.13045, -0.001)


class Silent:
    """A generator without noise."""

    def normal(self, _mean, _sigma, size):
        return np.zeros(size)


def test_at_rest_it_reads_gravity_up():
    force, gyro = imu_sample(Twist(), Twist(), IMU_XY, Silent())
    assert force == pytest.approx([0.0, 0.0, GRAVITY]) and gyro == pytest.approx([0.0, 0.0, 0.0])


def test_speeding_up_reads_the_acceleration():
    force, _ = imu_sample(Twist(0.4), Twist(0.5, -0.2), IMU_XY, Silent())
    assert force == pytest.approx([0.5, -0.2, GRAVITY])


def test_turning_reads_the_centripetal_and_tangential_terms():
    """At (0.5, 0) on a body turning at 1 rad/s: -w^2 r inward; dw/dt r sideways."""
    force, gyro = imu_sample(Twist(0.0, 0.0, 1.0), Twist(0.0, 0.0, 2.0), (0.5, 0.0), Silent())
    assert force == pytest.approx([-0.5, 1.0, GRAVITY]) and gyro[2] == pytest.approx(1.0)
    force, _ = imu_sample(Twist(0.4, 0.0, 0.5), Twist(), (0.0, 0.0), Silent())
    assert force[1] == pytest.approx(0.5 * 0.4)  # w x v: the curve's centripetal pull


def test_noise_is_small_and_seeded():
    """The stack's still checks (std / |g| below 1 %) pass on the noisy IMU at rest."""
    rng = default_rng(3)
    samples = np.array([imu_sample(Twist(), Twist(), IMU_XY, rng)[0] for _ in range(500)])
    norms = np.linalg.norm(samples, axis=1)
    assert norms.std() / norms.mean() < 0.01
    again = imu_sample(Twist(), Twist(), IMU_XY, default_rng(3))[0]
    assert again == pytest.approx(samples[0])


def test_the_body_keeps_its_acceleration():
    """Speeding up at 1 m/s^2: the body's accel; braking at 2 m/s^2; zero when steady."""
    body = Body(Pose2D(), BodyLimits(accel=1.0, decel=2.0, yaw_accel=2.0))
    body.step(0.0, Twist(1.0))
    body.step(0.1, Twist(1.0))
    assert body.accel.vx == pytest.approx(1.0)
    t = 0.1
    while body.twist.vx < 1.0:
        t += 0.1
        body.step(t, Twist(1.0))
    body.step(t + 0.1, Twist(1.0))
    assert body.accel.vx == pytest.approx(0.0)
    body.step(t + 0.2, Twist())
    assert body.accel.vx == pytest.approx(-2.0)
