"""Path controllers on kinematic simulations, avoidance biases, PID and MPPI details."""

from __future__ import annotations

import math

import numpy as np
import pytest
from sim import LOOP, OUT_AND_BACK, drive, noisy_route, route_of, step, straight_route

from x30_navigation.core.controllers import (
    CONTROLLERS,
    Bias,
    ControllerLimits,
    TrackRequest,
)
from x30_navigation.core.controllers.holonomic import HolonomicConfig
from x30_navigation.core.controllers.mppi import MppiConfig
from x30_navigation.core.controllers.pid import PidConfig
from x30_navigation.core.controllers.pure_pursuit import PurePursuitConfig
from x30_utils.math import Pose2D

LIMITS = ControllerLimits(max_vx=0.8, max_vy=0.4, max_wz=1.0, period_sec=0.05)


def make(name, config=None, limits=LIMITS):
    return CONTROLLERS.create(name, config, limits)


def test_every_builtin_is_registered():
    assert CONTROLLERS.names() == ['holonomic', 'lqr', 'mppi', 'pid', 'pure_pursuit', 'stanley']


@pytest.mark.parametrize('name', ['holonomic', 'lqr', 'mppi', 'pid', 'pure_pursuit', 'stanley'])
@pytest.mark.parametrize('corners', [LOOP, OUT_AND_BACK], ids=['loop', 'out_and_back'])
def test_controller_completes_the_route(name, corners):
    route = route_of(noisy_route(corners), spacing=0.1)
    finished, errors = drive(make(name), route)
    assert finished
    assert np.mean(errors) < 0.1 and max(errors) < 0.5


def run(controller, route, pose, seconds, speed=0.4, dt=0.05, offset=0.0):
    poses, commands, s = [], [], 0.0
    from x30_navigation.core.path.progress import RouteProgress

    progress = RouteProgress(route)
    progress.locate(pose)
    for _ in range(int(seconds / dt)):
        s = progress.update(pose, offset, 0.2)
        command = controller.update(TrackRequest(route, s, pose, speed, dt, offset))
        pose = step(pose, command.vx, command.vy, command.wz, dt)
        poses.append(pose)
        commands.append(command)
    return poses, commands


def test_holonomic_closes_cross_track_sideways_without_turning():
    route = route_of(straight_route(0.0, 20.0))
    poses, commands = run(make('holonomic'), route, Pose2D(0.0, 0.5, 0.0), 12.0)
    ys = [p.y for p in poses]
    assert abs(ys[-1]) < 0.02 and min(ys) > -0.01
    assert max(abs(p.yaw) for p in poses) < 0.02
    assert any(c.vy < -0.05 for c in commands)


def test_holonomic_backs_up_when_the_route_is_behind():
    route = route_of(straight_route(5.0, -5.0))
    controller = make('holonomic', HolonomicConfig(max_reverse_speed=0.15))
    first = controller.update(TrackRequest(route, 5.0, Pose2D(0.0, 0.0, 0.0), 0.4, 0.05))
    assert -0.15 - 1e-9 <= first.vx < 0.0 and abs(first.wz) > 0.5
    finished, errors = drive(controller, route, start=Pose2D(0.0, 0.0, 0.0))
    assert finished and max(errors) < 0.2


def test_holonomic_follows_the_offset_line_and_respects_the_robot():
    route = route_of(straight_route(0.0, 20.0))
    poses, _ = run(make('holonomic'), route, Pose2D(0.0, 0.0, 0.0), 12.0, offset=0.5)
    assert abs(poses[-1].y - 0.5) < 0.02 and max(abs(p.yaw) for p in poses) < 0.02
    narrow = ControllerLimits(max_vx=0.8, max_vy=0.1, max_wz=1.0)
    _, slow = run(make('holonomic', limits=narrow), route, Pose2D(0.0, 0.6, 0.0), 2.0)
    assert max(abs(c.vy) for c in slow) <= 0.1 + 1e-9


def test_holonomic_without_strafing_completes_the_loop():
    route = route_of(noisy_route(LOOP), spacing=0.1)
    diff = ControllerLimits(max_vx=0.8, max_vy=0.0, max_wz=1.0)
    finished, errors = drive(make('holonomic', limits=diff), route)
    assert finished and np.mean(errors) < 0.15


@pytest.mark.parametrize('name', ['holonomic', 'lqr', 'mppi', 'pid', 'pure_pursuit', 'stanley'])
def test_biases_have_one_meaning_in_every_controller(name):
    """N4: heading_offset (rad) turns the direction of travel left, yaw_rate_bias adds rad/s."""
    route = route_of(straight_route(0.0, 20.0))
    pose = Pose2D(2.0, 0.0, 0.0)
    plain = TrackRequest(route, 2.0, pose, 0.4, 0.05)
    base = make(name, MppiConfig(seed=3) if name == 'mppi' else None).update(plain)
    turned = make(name, MppiConfig(seed=3) if name == 'mppi' else None).update(
        TrackRequest(route, 2.0, pose, 0.4, 0.05, bias=Bias(heading_offset=0.5))
    )
    if name == 'holonomic':
        assert math.atan2(turned.vy, turned.vx) == pytest.approx(0.5, abs=1e-6)
    else:
        assert turned.wz > base.wz + 0.1
    biased = make(name, MppiConfig(seed=3) if name == 'mppi' else None).update(
        TrackRequest(route, 2.0, pose, 0.4, 0.05, bias=Bias(yaw_rate_bias=0.3))
    )
    assert biased.wz == pytest.approx(base.wz + 0.3, abs=1e-6)


def test_pid_integral_is_clamped_and_recovers():
    """N2: a long saturated error does not wind the integral up."""
    route = route_of(straight_route(0.0, 50.0))
    pid = make('pid', PidConfig(ki=2.0, integral_limit=0.3))
    away = Pose2D(5.0, 0.0, math.pi / 2)
    for _ in range(200):
        pid.update(TrackRequest(route, 5.0, away, 0.4, 0.05))
    assert abs(pid.integral) <= 0.3 + 1e-9
    aligned = pid.update(TrackRequest(route, 5.0, Pose2D(5.0, 0.0, -0.3), 0.4, 0.05))
    assert aligned.wz > 0.0


def test_mppi_is_seeded_and_warm_started():
    """N2: same seed, same output; the plan is shifted by one step for the next tick."""
    route = route_of(straight_route(0.0, 20.0))
    request = TrackRequest(route, 1.0, Pose2D(1.0, 0.2, 0.1), 0.4, 0.05)
    a, b = make('mppi', MppiConfig(seed=7)), make('mppi', MppiConfig(seed=7))
    assert a.update(request) == b.update(request)
    plan = a.plan.copy()
    assert plan[-1] == plan[-2]
    a.update(request)
    assert np.allclose(a.plan[:-2], plan[1:-1], atol=0.5)


def test_mppi_steers_around_scan_obstacles():
    """N2: obstacle points of the safety scan (robot frame) push the rollouts away."""
    route = route_of(straight_route(0.0, 20.0))
    pose = Pose2D(1.0, 0.0, 0.0)
    wall = np.array([(1.0, y) for y in np.linspace(-0.6, 0.15, 16)])
    free = make('mppi', MppiConfig(seed=1)).update(TrackRequest(route, 1.0, pose, 0.4, 0.05))
    blocked = make('mppi', MppiConfig(seed=1, obstacle_weight=50.0)).update(
        TrackRequest(route, 1.0, pose, 0.4, 0.05, obstacles=wall)
    )
    assert blocked.wz > free.wz + 0.2


def test_pure_pursuit_steers_toward_the_carrot():
    route = route_of(straight_route(0.0, 20.0))
    left = make('pure_pursuit', PurePursuitConfig(lookahead=1.0)).update(
        TrackRequest(route, 2.0, Pose2D(2.0, -0.3, 0.0), 0.4, 0.05)
    )
    assert left.wz > 0.0 and left.vx == pytest.approx(0.4)
