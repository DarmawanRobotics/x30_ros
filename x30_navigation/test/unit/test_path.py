"""Route geometry, processing, speed profile and progress."""

from __future__ import annotations

import math

import numpy as np
import pytest
from sim import LOOP, OUT_AND_BACK, noisy_route, route_of, straight_route

from x30_navigation.core.path.processing import process_route, resample, smooth
from x30_navigation.core.path.profile import ProfileLimits, speed_profile
from x30_navigation.core.path.progress import RouteProgress
from x30_navigation.core.path.route import Route
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D
from x30_utils.safety import stopping_speed

LIMITS = ProfileLimits(
    cruise=0.4, min_speed=0.1, approach=0.15, lateral_accel=0.3, accel=0.3, decel=0.3
)


def test_route_needs_two_points_and_length():
    with pytest.raises(ValueError):
        Route.from_points([RoutePoint(0.0, 0.0)])
    with pytest.raises(ValueError):
        Route.from_points([RoutePoint(1.0, 1.0), RoutePoint(1.0, 1.0)])
    with pytest.raises(ValueError):
        Route.from_points([RoutePoint(0.0, 0.0), RoutePoint(math.nan, 1.0)])


def test_route_interpolation_and_shift():
    route = route_of([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    assert route.length == pytest.approx(4.0)
    assert route.point_at(1.0) == pytest.approx((1.0, 0.0))
    assert route.point_at(3.0) == pytest.approx((2.0, 1.0))
    assert route.point_at(-1.0) == pytest.approx((0.0, 0.0))
    assert route.shifted_point(1.0, 0.5) == pytest.approx((1.0, 0.5))
    assert route.lateral(Pose2D(1.0, -0.3), 1.0) == pytest.approx(-0.3)
    assert route.index_at(2.0) == 1 and route.index_at(1.99) == 0


def test_resample_and_smooth_keep_the_ends():
    pts = resample(np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]), 0.1)
    gaps = np.hypot(*np.diff(pts, axis=0).T)
    assert tuple(pts[0]) == (0.0, 0.0) and tuple(pts[-1]) == (1.0, 1.0)
    assert gaps.max() <= 0.1 + 1e-9
    smoothed = smooth(pts, 0.05, 0.4)
    assert tuple(smoothed[0]) == (0.0, 0.0) and tuple(smoothed[-1]) == (1.0, 1.0)


def test_smoothing_keeps_stop_points_and_reduces_jitter():
    raw = noisy_route([(0, 0), (12, 0)], noise=0.03)
    raw[30] = RoutePoint(raw[30].x, raw[30].y, math.pi / 2, 5.0)
    route, deviation = process_route(raw, 0.1, 0.05, 0.4)
    assert raw[30] in route and route[0] == raw[0] and route[-1] == raw[-1]
    assert deviation < 0.15
    ys = np.array([p.y for p in route])
    assert np.abs(np.diff(ys)).max() < np.abs(np.diff([p.y for p in raw])).max() / 3.0


def test_close_stop_points_are_all_kept():
    """F12: stop points 2 cm apart both survive processing."""
    raw = [RoutePoint(0.1 * i, 0.0) for i in range(30)]
    raw[10] = RoutePoint(1.0, 0.0, 0.0, 3.0)
    raw.insert(11, RoutePoint(1.02, 0.0, 1.0, 4.0))
    route, _ = process_route(raw, 0.1, 0.05, 0.4)
    stops = [p for p in route if p.is_stop]
    assert stops == [raw[10], raw[11]]


def test_profile_brakes_into_stops_with_the_stopping_model():
    route = route_of(straight_route(0.0, 10.0), stops={50: 5.0})
    v = speed_profile(route, LIMITS)
    assert v[50] <= LIMITS.approach + 1e-9 and v[-1] <= LIMITS.approach + 1e-9
    assert v[48] < v[45] <= LIMITS.cruise
    assert max(v) <= LIMITS.cruise + 1e-9
    reaction = ProfileLimits(**{**LIMITS.__dict__, 'reaction_time': 0.5})
    slower = speed_profile(route, reaction)
    assert slower[46] < v[46]
    room = route.s[50] - route.s[46] + LIMITS.approach**2 / (2 * LIMITS.decel)
    assert slower[46] <= stopping_speed(room, LIMITS.decel, 0.5) + 1e-9


def test_profile_forward_pass_limits_acceleration():
    """N5: leaving a stop the speed grows at most like sqrt(v^2 + 2 a ds)."""
    route = route_of(straight_route(0.0, 10.0), stops={20: 5.0})
    v = speed_profile(route, LIMITS)
    ds = np.diff(route.s)
    grow = v[1:] ** 2 - v[:-1] ** 2
    assert np.all(grow <= 2.0 * LIMITS.accel * ds + 1e-9)
    assert v[21] < 0.3 and v[0] <= max(LIMITS.min_speed, LIMITS.approach) + 1e-9


def test_profile_curvature_cap():
    route = route_of(noisy_route(LOOP, noise=0.0), spacing=0.1)
    v = speed_profile(route, LIMITS)
    k = np.abs(route.curvature)
    tight = k > 1.0
    assert np.all(v[tight] <= np.sqrt(LIMITS.lateral_accel / k[tight]) + 1e-6)
    assert v.min() >= 0.0


def test_progress_locates_and_is_monotonic():
    route = route_of(straight_route(0.0, 10.0))
    progress = RouteProgress(route)
    assert progress.locate(Pose2D(3.0, 0.2, 0.0)) == pytest.approx(3.0, abs=1e-6)
    progress.update(Pose2D(2.0, 0.0, 0.0), 0.0, 1.0)
    assert progress.s == pytest.approx(3.0)
    progress.update(Pose2D(3.5, 0.0, 0.0), 0.0, 0.1)
    assert progress.s == pytest.approx(3.1)


def test_shifted_progress_stays_on_its_leg():
    """F5: shifted 0.8 m toward the parallel return leg, progress stays on the outbound one."""
    route = route_of(OUT_AND_BACK)
    progress = RouteProgress(route)
    progress.locate(Pose2D(0.0, 0.0, 0.0))
    for x in np.arange(0.1, 9.0, 0.05):
        progress.update(Pose2D(float(x), 0.8, 0.0), 0.8, 0.2)
        assert progress.s == pytest.approx(x, abs=0.06)
    located = RouteProgress(route).locate(Pose2D(5.0, 0.8, 0.0))
    assert located == pytest.approx(5.0, abs=0.05)
    back = RouteProgress(route).locate(Pose2D(5.0, 0.8, math.pi))
    assert back > route.length / 2.0
