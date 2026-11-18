from __future__ import annotations

import math

import numpy as np
import pytest

from x30_localization.core.config import EstimatorConfig
from x30_localization.core.estimator import MapOdomEstimator, State, jacobian
from x30_localization.core.motion import MotionCounter
from x30_localization.core.sources.base import Measurement
from x30_utils.math import Pose2D, default_rng

FIX = np.diag([0.05**2, 0.05**2, math.radians(2.0) ** 2])


def measurement(x, y, yaw, cov=FIX, stamp=1.0, source='tag:a'):
    return Measurement(Pose2D(x, y, yaw), cov, stamp, source)


ORIGIN = Pose2D()


def localized(map_odom=ORIGIN, odom_base=ORIGIN, cov=FIX):
    est = MapOdomEstimator(EstimatorConfig())
    est.predict(odom_base)
    base = map_odom @ odom_base
    est.set(measurement(base.x, base.y, base.yaw, cov), odom_base)
    return est


def test_set_places_the_robot_on_the_measurement():
    odom_base = Pose2D(4.0, -2.0, math.radians(120.0))
    est = MapOdomEstimator()
    est.set(measurement(1.0, 2.0, math.radians(-30.0)), odom_base)
    base = est.map_odom @ odom_base
    assert base.to_array() == pytest.approx([1.0, 2.0, math.radians(-30.0)])
    assert est.state is State.LOCALIZED and est.source == 'tag:a'
    # the robot covariance is the measurement covariance, whatever the lever arm
    est.predict(odom_base)
    assert np.allclose(est.base_covariance(), FIX)


def test_predict_grows_with_distance_not_time():
    est = localized()
    s0 = est.sigma_xy
    for _ in range(100):  # standing still: no growth
        est.predict(Pose2D())
    assert est.sigma_xy == pytest.approx(s0)
    # along the direction of travel the variance grows by q per metre (the heading
    # uncertainty adds lateral variance on top, checked by the next test)
    est.predict(Pose2D(1.0, 0.0, 0.0), distance=1.0)
    v1 = est.base_covariance()[0, 0]
    est.predict(Pose2D(4.0, 0.0, 0.0), distance=3.0)
    v4 = est.base_covariance()[0, 0]
    q = EstimatorConfig().sigma_xy_per_sqrt_m ** 2
    assert v1 == pytest.approx(FIX[0, 0] + q, rel=1e-6)
    assert v4 == pytest.approx(FIX[0, 0] + 4 * q, rel=1e-6)
    assert est.base_covariance()[1, 1] > v4  # lateral: heading uncertainty times distance
    yaw0 = est.sigma_yaw
    est.predict(Pose2D(4.0, 0.0, 1.0), rotation=1.0)
    assert est.sigma_yaw > yaw0


def test_heading_uncertainty_far_from_origin_moves_the_robot_sideways():
    """Yaw variance of map->odom must show up as lateral variance of a far robot."""
    est = MapOdomEstimator()
    odom_base = Pose2D(10.0, 0.0, 0.0)
    est.set(measurement(10.0, 0.0, 0.0), odom_base)
    est.predict(Pose2D(10.0, 0.0, 0.0))
    est.predict(Pose2D(10.0, 0.0, 2.0), rotation=2.0)  # turn in place: yaw drift only
    cov = est.base_covariance()
    assert cov[0, 0] == pytest.approx(FIX[0, 0], rel=1e-6)
    assert cov[1, 1] == pytest.approx(FIX[1, 1], rel=1e-6)
    assert cov[2, 2] > FIX[2, 2]


def test_update_shrinks_covariance_and_moves_toward_the_fix():
    est = localized()
    est.predict(Pose2D(3.0, 0.0, 0.0), distance=3.0)
    before = est.sigma_xy
    result = est.update(measurement(3.05, 0.02, 0.01), Pose2D(3.0, 0.0, 0.0))
    assert result.accepted
    assert est.sigma_xy < before
    base = est.base_pose()
    assert 3.0 < base.x < 3.05 and 0.0 < base.y < 0.02


def test_gate_rejects_outliers_and_accepts_consistent_fixes_far_from_origin():
    """Review #1: the gate judges the robot pose, not map->odom."""
    # far from the odom origin a 2 deg map->odom rotation moves map->odom's origin a lot
    odom_base = Pose2D(50.0, 0.0, 0.0)
    est = localized(Pose2D(), odom_base)
    est.predict(Pose2D(51.0, 0.0, 0.0), distance=1.0)
    good = est.update(measurement(51.03, -0.02, math.radians(0.5)), Pose2D(51.0, 0.0, 0.0))
    assert good.accepted, good.describe()
    # 10 deg rotation about the odom origin moves the robot 8.9 m: rejected
    est2 = localized(Pose2D(), odom_base)
    far = Pose2D(0.0, 0.0, math.radians(10.0)) @ odom_base
    bad = est2.update(measurement(far.x, far.y, far.yaw), odom_base)
    assert not bad.accepted
    assert est2.map_odom.to_array() == pytest.approx([0.0, 0.0, 0.0])


def test_gate_grows_with_uncertainty():
    """The same 1 m offset is rejected when certain and accepted after a long drive."""
    est = localized()
    assert not est.update(measurement(1.0, 0.0, 0.0), Pose2D()).accepted
    for i in range(1, 201):
        est.predict(Pose2D(float(i), 0.0, 0.0), distance=1.0)
    assert est.state is State.UNCERTAIN or est.sigma_xy > 0.4
    result = est.update(measurement(201.0, 0.0, 0.0), Pose2D(200.0, 0.0, 0.0))
    assert result.accepted, result.describe()


def test_joseph_update_keeps_covariance_symmetric_positive():
    est = localized(Pose2D(1.0, 2.0, 0.7), Pose2D(-3.0, 5.0, -2.0))
    rng = default_rng(1)
    odom = Pose2D(-3.0, 5.0, -2.0)
    for _ in range(50):
        odom = odom @ Pose2D(0.3, 0.0, 0.1)
        est.predict(odom, distance=0.3, rotation=0.1)
        truth = Pose2D(1.0, 2.0, 0.7) @ odom
        noise = rng.normal(0.0, [0.03, 0.03, 0.01])
        est.update(measurement(truth.x + noise[0], truth.y + noise[1], truth.yaw + noise[2]), odom)
        assert np.allclose(est.covariance, est.covariance.T)
        assert np.all(np.linalg.eigvalsh(est.covariance) > 0.0)
    assert est.map_odom.to_array() == pytest.approx([1.0, 2.0, 0.7], abs=0.05)


def test_jacobian_matches_numeric_derivative():
    mo, ob = Pose2D(1.0, -2.0, 0.4), Pose2D(3.0, 1.5, -1.1)
    h = jacobian(mo, ob)
    eps = 1e-6
    for k in range(3):
        d = np.zeros(3)
        d[k] = eps
        plus = (Pose2D.from_array(mo.to_array() + d) @ ob).to_array()
        minus = (Pose2D.from_array(mo.to_array() - d) @ ob).to_array()
        assert (plus - minus) / (2 * eps) == pytest.approx(h[:, k], abs=1e-6)


def test_reset_and_state_none():
    est = localized()
    est.reset()
    assert est.state is State.NONE and est.base_pose() is None
    assert math.isinf(est.sigma_xy)
    with pytest.raises(RuntimeError):
        est.update(measurement(0.0, 0.0, 0.0), Pose2D())


def test_motion_counter_ignores_jitter_and_counts_steps():
    counter = MotionCounter(0.05, math.radians(2.0))
    assert counter.update(Pose2D()) == (0.0, 0.0)
    for i in range(100):  # 1 cm jitter forever: nothing
        assert counter.update(Pose2D(0.01 * (-1) ** i, 0.0, 0.0)) == (0.0, 0.0)
    distance, rotation = counter.update(Pose2D(0.2, 0.0, 0.0))
    assert distance == pytest.approx(0.2) and rotation == 0.0
    distance, rotation = counter.update(Pose2D(0.2, 0.0, 0.5))
    assert distance == pytest.approx(0.0) and rotation == pytest.approx(0.5)
