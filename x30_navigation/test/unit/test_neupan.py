"""The NeuPAN controller: its checks before it builds, then on routes with the X30's model.

The tests driving the robot need NeuPAN with the X30 patches (tools/install_neupan.sh) and are
skipped without it.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest
from sim import route_of, straight_route

from x30_navigation.core.controllers import CONTROLLERS, ControllerLimits, TrackRequest
from x30_navigation.core.controllers.neupan_mpc import NeupanConfig, body_vertices
from x30_navigation.core.dune_model import DuneRecord, polygon_distance
from x30_navigation.core.path.progress import RouteProgress
from x30_utils.math import Pose2D

MODEL = str(Path(__file__).resolve().parents[2] / 'models' / 'dune_x30.pth')
X30_BODY = ((0.5, 0.235), (-0.5, 0.235), (-0.5, -0.235), (0.5, -0.235))
X30 = ControllerLimits(
    max_vx=0.75,
    max_vy=0.15,
    max_wz=0.45,
    period_sec=0.05,
    max_accel=0.5,
    max_yaw_accel=1.5,
    footprint=X30_BODY,
)


def fake_model(tmp_path, footprint=X30_BODY, span=8.0) -> str:
    """Return a checkpoint whose record is valid (its weights are not)."""
    checkpoint = tmp_path / 'dune.pth'
    checkpoint.write_bytes(b'not weights')
    vertices = np.asarray(footprint, dtype=float).T.tolist()
    DuneRecord(vertices, [-span, -span, span, span]).save(str(checkpoint))
    return str(checkpoint)


def test_neupan_is_registered_with_its_parameters():
    assert 'neupan' in CONTROLLERS.names()
    assert CONTROLLERS.get('neupan').avoids and CONTROLLERS.get('neupan').strafes
    assert NeupanConfig().validate() == []
    assert NeupanConfig(min_clearance=0.5, max_clearance=0.2).validate()


def test_neupan_needs_a_model_of_the_footprint(tmp_path):
    with pytest.raises(ValueError, match='no DUNE checkpoint set'):
        CONTROLLERS.create('neupan', NeupanConfig(), X30)
    other = fake_model(tmp_path, footprint=((0.4, 0.3), (-0.4, 0.3), (-0.4, -0.3), (0.4, -0.3)))
    with pytest.raises(ValueError, match='another footprint'):
        CONTROLLERS.create('neupan', NeupanConfig(dune_checkpoint=other), X30)


def test_neupan_keeps_the_points_where_the_model_was_trained(tmp_path):
    small = fake_model(tmp_path, span=3.0)
    with pytest.raises(ValueError, match='trained for'):
        CONTROLLERS.create('neupan', NeupanConfig(dune_checkpoint=small), X30)


def test_neupan_says_how_to_install_it(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'neupan', None)
    with pytest.raises(ImportError, match=r'install_neupan\.sh'):
        CONTROLLERS.create('neupan', NeupanConfig(dune_checkpoint=fake_model(tmp_path)), X30)


def need_neupan():
    """Skip unless NeuPAN with the holonomic kinematics is installed."""
    pytest.importorskip('torch')
    robot = pytest.importorskip('neupan.robot').robot
    if not hasattr(robot(kinematics='diff', length=1.0, width=0.5), 'control_dim'):
        pytest.skip('NeuPAN without the X30 patches')


def drive(route, obstacles=(), limits=X30, seconds=40.0, speed=0.4, start=None, config=None):
    """Follow the route with NeuPAN: poses, commands, least clearance, widest lateral, finished."""
    need_neupan()
    controller = CONTROLLERS.create(
        'neupan', config or NeupanConfig(dune_checkpoint=MODEL), limits
    )
    world = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    vertices = body_vertices(limits)
    progress = RouteProgress(route)
    pose = start or Pose2D(float(route.xy[0, 0]), float(route.xy[0, 1]), float(route.yaw[0]))
    progress.locate(pose)
    poses, commands, clearance, lateral, dt = [], [], math.inf, 0.0, 0.05
    for _ in range(int(seconds / dt)):
        s = progress.update(pose, 0.0, 0.2)
        lateral = max(lateral, abs(progress.lateral(pose)))
        if progress.remaining < 0.2:
            return poses, commands, clearance, lateral, True
        local = pose.inverse().apply(world) if len(world) else np.zeros((0, 2))
        command = controller.update(TrackRequest(route, s, pose, speed, dt, obstacles=local))
        yaw = pose.yaw + command.wz * dt
        c, si = math.cos(yaw), math.sin(yaw)
        pose = Pose2D(
            pose.x + (command.vx * c - command.vy * si) * dt,
            pose.y + (command.vx * si + command.vy * c) * dt,
            yaw,
        )
        poses.append(pose)
        commands.append(command)
        if len(world):
            clearance = min(
                clearance, polygon_distance(vertices, pose.inverse().apply(world)).min()
            )
    return poses, commands, clearance, lateral, False


def test_neupan_follows_a_straight_route_at_speed_along_it():
    poses, commands, _, lateral, finished = drive(route_of(straight_route(0.0, 6.0)))
    assert finished
    assert lateral < 0.05 and max(abs(p.yaw) for p in poses) < 0.05
    assert np.median([c.vx for c in commands]) == pytest.approx(0.4, abs=0.03)


def test_neupan_turns_the_body_with_the_route():
    route = route_of([*straight_route(0.0, 4.0), *[(4.0, 0.1 * i) for i in range(1, 41)]])
    poses, _, _, lateral, finished = drive(route)
    assert finished
    assert poses[-1].yaw == pytest.approx(math.pi / 2.0, abs=0.25)
    assert lateral < 0.4  # the corner is cut, never left


def test_neupan_steps_around_an_obstacle_on_the_route():
    pole = [(3.0 + 0.1 * math.cos(a), 0.1 + 0.1 * math.sin(a)) for a in np.linspace(0, 6.2, 12)]
    _, _, clearance, lateral, finished = drive(route_of(straight_route(0.0, 6.0)), pole)
    assert finished
    assert clearance > 0.1  # min_clearance
    assert lateral > 0.3  # it went around, not through


def test_neupan_passes_a_narrow_passage_without_stalling():
    walls = [(x, y) for x in np.linspace(1.0, 5.0, 41) for y in (-0.5, 0.5)]  # 1 m wide
    _, commands, clearance, _, finished = drive(route_of(straight_route(0.0, 6.0)), walls)
    assert finished
    assert clearance > 0.2
    assert np.median([c.vx for c in commands]) == pytest.approx(0.4, abs=0.03)


def test_neupan_never_walks_into_a_wall_across_the_route():
    wall = [(3.0, y) for y in np.linspace(-1.5, 1.5, 31)]
    poses, _, clearance, lateral, finished = drive(
        route_of(straight_route(0.0, 6.0)), wall, seconds=20.0
    )
    assert not finished
    assert clearance > 0.1  # min_clearance
    assert max(p.x for p in poses) < 3.0 - 0.5  # never through: the body's front is 0.5 m
    assert lateral < 1.5  # looking for a way around, the follower's off_path_stop never reached


def test_neupan_without_strafing_plans_like_a_differential_robot():
    diff = ControllerLimits(0.75, 0.0, 0.45, 0.05, 0.5, 1.5, X30_BODY)
    _, commands, _, _, finished = drive(route_of(straight_route(0.0, 4.0)), limits=diff)
    assert finished
    assert all(c.vy == 0.0 for c in commands)
