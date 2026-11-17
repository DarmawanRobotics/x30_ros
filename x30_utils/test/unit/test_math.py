from __future__ import annotations

import math

import numpy as np
import pytest

from x30_utils.math import (
    Cusum,
    Ema,
    Hold,
    LegacyGenerator,
    Pose2D,
    RateMeter,
    SlewLimiter,
    angle_diff,
    arc_lengths,
    chi2_threshold,
    clamp,
    compose,
    default_rng,
    default_rngs,
    flatten,
    inverse,
    kth_smallest,
    make_transform,
    mean_angle,
    quat_from_matrix,
    quat_from_rpy,
    quat_to_matrix,
    rotation_aligning,
    rpy_from_quat,
    tilt_of,
    transform_points,
    wrap_angle,
    yaw_of,
)


def test_wrap_angle():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-3 * math.pi) == pytest.approx(-math.pi)
    assert angle_diff(0.1, -0.1) == pytest.approx(0.2)
    assert angle_diff(-math.pi + 0.05, math.pi - 0.05) == pytest.approx(0.1)


def test_mean_angle_across_pi():
    assert abs(wrap_angle(mean_angle([math.pi - 0.1, -math.pi + 0.1]) - math.pi)) < 1e-9


def test_quaternion_roundtrip():
    for rpy in [(0.1, -0.2, 0.3), (0.0, 0.0, -2.9), (-1.0, 0.5, 3.0)]:
        q = quat_from_rpy(*rpy)
        assert rpy_from_quat(*q) == pytest.approx(rpy, abs=1e-9)
        assert quat_from_matrix(quat_to_matrix(q)) == pytest.approx(
            q if q[3] >= 0 else tuple(-c for c in q), abs=1e-9
        )


def test_transform_inverse_and_compose():
    a = make_transform((1.0, 2.0, 0.5), quat_from_rpy(0.1, 0.2, 0.3))
    b = make_transform((-0.4, 0.0, 1.0), quat_from_rpy(-0.3, 0.0, 1.2))
    assert np.allclose(compose(a, inverse(a)), np.eye(4))
    p = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, -1.0]])
    assert np.allclose(
        transform_points(compose(a, b), p), transform_points(a, transform_points(b, p))
    )


def test_flatten_keeps_xy_yaw():
    m = make_transform((1.0, 2.0, 3.0), quat_from_rpy(0.2, -0.1, 0.7))
    f = flatten(m)
    assert f[2, 3] == 0.0
    assert yaw_of(f) == pytest.approx(yaw_of(m))
    assert tilt_of(f) == pytest.approx((0.0, 0.0), abs=1e-12)


def test_rotation_aligning_levels_gravity():
    tilted = np.array([0.1, -0.05, 0.99])
    r = rotation_aligning(tilted, [0.0, 0.0, 1.0])
    assert np.allclose(r @ (tilted / np.linalg.norm(tilted)), [0, 0, 1])
    assert np.allclose(rotation_aligning([0, 0, 1], [0, 0, -1]) @ [0, 0, 1], [0, 0, -1])


def test_pose2d_algebra():
    a, b = Pose2D(1.0, 2.0, 0.5), Pose2D(-0.3, 0.4, -1.2)
    assert (a @ a.inverse()).to_array() == pytest.approx([0, 0, 0], abs=1e-12)
    assert np.allclose((a @ b).to_matrix(), a.to_matrix() @ b.to_matrix())
    assert a.between(a @ b).to_array() == pytest.approx(b.to_array())
    assert a.apply([[1.0, 0.0]])[0] == pytest.approx([1 + math.cos(0.5), 2 + math.sin(0.5)])


def test_clamp_rejects_nan():
    assert clamp(float('nan'), -1.0, 1.0) == 0.0
    assert clamp(5.0, -1.0, 1.0) == 1.0


def test_arc_lengths():
    assert arc_lengths([[0, 0], [3, 4], [3, 5]]) == pytest.approx([0, 5, 6])


def test_slew_limiter():
    slew = SlewLimiter(rise=1.0, fall=2.0)
    assert slew.step(1.0, 0.1) == pytest.approx(0.1)
    slew.reset(1.0)
    assert slew.step(0.0, 0.1) == pytest.approx(0.8)
    assert slew.step(float('nan'), 0.1) == pytest.approx(0.6)
    with pytest.raises(ValueError):
        SlewLimiter(-1.0)


def test_ema_rate_cusum_hold():
    ema = Ema(0.5)
    ema.update(0.0)
    assert ema.update(2.0) == 1.0
    meter = RateMeter(5.0)
    for i in range(11):
        meter.tick(i * 0.1)
    assert meter.rate(1.0) == pytest.approx(10.0)
    assert meter.rate(5.0) == 0.0
    cusum = Cusum(drift=1.0, threshold=3.0)
    assert not any(cusum.update(1.5) for _ in range(6))
    assert cusum.update(1.5)
    hold = Hold(on_sec=1.0, off_sec=0.5)
    assert not hold.update(True, 0.0)
    assert hold.update(True, 1.0)
    assert hold.update(False, 1.2)
    assert not hold.update(False, 1.7)


def test_stats():
    assert chi2_threshold(3) == pytest.approx(11.345)
    assert kth_smallest([5, 1, 3, 2], 2) == 2
    assert math.isinf(kth_smallest([1.0], 3))


def test_rate_window_ends_now_not_at_the_last_message():
    meter = RateMeter(window_sec=1.0)
    for i in range(10):
        meter.tick(i * 0.1)
    meter.tick(1.5)
    assert meter.rate(1.8) == pytest.approx(2 / 0.7)


def test_cusum_cap_bounds_the_recovery():
    cusum = Cusum(drift=1.0, threshold=5.0, cap=10.0)
    for _ in range(100):
        cusum.update(11.0)
    assert cusum.value == 10.0
    alarms = [cusum.update(0.0) for _ in range(5)]
    assert alarms[0] and not alarms[-1]


@pytest.mark.parametrize('make', [default_rng, LegacyGenerator])
def test_random_generators_share_the_api(make):
    a, b = make(7), make(7)
    assert np.array_equal(a.normal(0.0, 1.0, 5), b.normal(0.0, 1.0, 5))
    values = a.integers(3, size=200)
    assert values.min() >= 0 and values.max() <= 2
    assert set(a.integers(1, 2, size=50, endpoint=True).tolist()) <= {1, 2}
    small = a.integers(0, 200, size=10, dtype=np.uint8)
    assert small.dtype == np.uint8 and a.integers(5, size=3).dtype == np.int64
    assert 0.0 <= float(a.random()) < 1.0 and a.uniform(2.0, 3.0, (2, 2)).shape == (2, 2)
    assert a.standard_normal(4).shape == (4,) and len(a.choice([1, 2, 3], 2, replace=False)) == 2
    assert sorted(a.permutation(5).tolist()) == [0, 1, 2, 3, 4]
    x = np.arange(6)
    a.shuffle(x)
    assert sorted(x.tolist()) == list(range(6))
    assert a.exponential(1.0, 3).shape == (3,)


def _streams(generators) -> list:
    return [tuple(g.integers(0, 2**31, size=4).tolist()) for g in generators]


@pytest.mark.parametrize('seed_sequence', [True, False])
def test_default_rngs_are_independent_and_reproducible(seed_sequence, monkeypatch):
    if not seed_sequence:  # the numpy < 1.17 path
        monkeypatch.delattr(np.random, 'SeedSequence', raising=False)
    first = default_rngs(42, 4)
    assert len(first) == 4
    streams = _streams(first)
    assert len(set(streams)) == 4  # every child draws its own stream
    assert _streams(default_rngs(42, 4)) == streams  # same seed, same children
    assert _streams(default_rngs(43, 4)) != streams
    assert _streams(default_rngs(42, 6))[:4] == streams  # more children keep the first ones
    assert len(set(_streams(default_rngs(None, 3)))) == 3
    assert len(set(_streams(default_rngs(2**40 + 7, 2)))) == 2
    assert default_rngs(1, 0) == []


def test_default_rngs_reject_negative_seeds(monkeypatch):
    monkeypatch.delattr(np.random, 'SeedSequence', raising=False)
    with pytest.raises(ValueError):
        default_rngs(-1, 2)
