"""bench_neupan: time NeuPAN's solves on this computer as the path follower makes them (no ROS)."""

# $X30_PYTHON $(rospack find x30_navigation)/scripts/bench_neupan [--threads 2]
# The neupan controller, built from the robot profile and navigation.yaml as the follower builds
# it, walks a route at the follower's rate twice: in the open, and in a 3 m corridor past a box
# beside the route. Every control tick with a solve is timed. Exit 1 when the slow ones (95 %)
# take longer than NeuPAN's step: its commands would come late. The robot does not move; on the
# navigation host it runs beside the stack. Needs NeuPAN (tools/install_neupan.sh).

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers import CONTROLLERS, ControllerLimits, TrackRequest
from x30_navigation.core.controllers.neupan_mpc import NeupanConfig
from x30_navigation.core.path.progress import RouteProgress
from x30_navigation.core.path.route import Route
from x30_utils.config import build, flatten
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile

WARMUP = 3  # solves left out: NeuPAN sets itself up in the first ones
ROUTE = Route.from_points([RoutePoint(0.1 * i, 0.0) for i in range(121)])  # 12 m along x


def package(name: str) -> Path:
    """Return a package's directory: rospkg's, else the one beside this source tree."""
    beside = Path(__file__).resolve().parents[3] / name
    try:
        import rospkg
    except ImportError:
        return beside
    try:
        return Path(rospkg.RosPack().get_path(name))
    except rospkg.ResourceNotFound:
        return beside


def line(x0: float, x1: float, y: float, step: float = 0.05) -> np.ndarray:
    """Return points along x at y."""
    xs = np.arange(x0, x1 + 1e-9, step)
    return np.column_stack([xs, np.full_like(xs, y)])


def box(cx: float, cy: float, half: float, step: float = 0.05) -> np.ndarray:
    """Return points on the sides of a square."""
    edge = np.arange(-half, half + 1e-9, step)
    sides = [np.column_stack([edge, np.full_like(edge, side * half)]) for side in (-1, 1)]
    sides += [np.column_stack([np.full_like(edge, side * half), edge]) for side in (-1, 1)]
    return np.vstack(sides) + np.array([cx, cy])


SCENES = {
    'open route': np.zeros((0, 2)),
    'corridor, box': np.vstack(
        [line(-2.0, 14.0, -1.5), line(-2.0, 14.0, 1.5), box(3.0, 0.35, 0.2)]
    ),
}


def read_yaml(path: str) -> dict:
    """Return a YAML file's mapping."""
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def setup(args: argparse.Namespace):
    """Return the controller's limits and parameters, the control period and the cruise speed."""
    robot = build(RobotProfile, flatten(read_yaml(args.robot_params)))
    node = read_yaml(args.params).get('path_follower_node') or {}
    follower = build(FollowerConfig, flatten(node))
    values = flatten(node.get('neupan') or {})
    values['dune_checkpoint'] = args.dune
    if args.threads is not None:
        values['threads'] = args.threads
    config = build(NeupanConfig, values)
    lim, period = robot.limits, 1.0 / follower.control_rate
    limits = ControllerLimits(
        max_vx=lim.max_vx,
        max_vy=lim.max_vy if robot.can_strafe else 0.0,
        max_wz=lim.max_wz,
        period_sec=period,
        max_accel=lim.max_accel,
        max_yaw_accel=lim.max_yaw_accel,
        footprint=tuple(tuple(float(v) for v in c) for c in robot.footprint.corners()),
    )
    return limits, config, period, follower.speed.cruise


def walk(controller, obstacles: np.ndarray, period: float, speed: float, solves: int) -> list:
    """Return the seconds of every tick that solved, the robot moving as commanded."""
    progress, pose, times = RouteProgress(ROUTE), Pose2D(0.0, 0.0, 0.0), []
    progress.locate(pose)
    controller.reset()
    for _ in range(solves * 20):
        s = progress.update(pose, 0.0, 0.2)
        if progress.remaining < 0.3:  # the end: walk it again
            pose = Pose2D(0.0, 0.0, 0.0)
            progress.locate(pose)
            controller.reset()
            continue
        local = pose.inverse().apply(obstacles) if len(obstacles) else np.zeros((0, 2))
        start = time.perf_counter()
        command = controller.update(TrackRequest(ROUTE, s, pose, speed, period, obstacles=local))
        if controller.since_solve == 0.0:
            times.append(time.perf_counter() - start)
            if len(times) >= solves:
                break
        yaw = pose.yaw + command.wz * period
        c, si = math.cos(yaw), math.sin(yaw)
        x = pose.x + (command.vx * c - command.vy * si) * period
        pose = Pose2D(x, pose.y + (command.vx * si + command.vy * c) * period, yaw)
    return times


def parse(argv: list) -> argparse.Namespace:
    """Return the command line options."""
    navigation, bringup = package('x30_navigation'), package('x30_bringup')
    parser = argparse.ArgumentParser(prog='bench_neupan', description=__doc__.splitlines()[0])
    parser.add_argument('--robot-params', default=str(bringup / 'config' / 'robot.yaml'))
    parser.add_argument('--params', default=str(navigation / 'config' / 'navigation.yaml'))
    parser.add_argument('--dune', default=str(navigation / 'models' / 'dune_x30.pth'))
    parser.add_argument('--threads', type=int, help="torch threads (default: navigation.yaml's)")
    parser.add_argument('--solves', type=int, default=100, help='solves per route')
    return parser.parse_args(argv)


def main(argv: list | None = None) -> int:
    """Time the solves and say whether they fit; 0 fits, 1 too slow, 2 cannot build NeuPAN."""
    args = parse(sys.argv[1:] if argv is None else argv)
    limits, config, period, speed = setup(args)
    try:
        controller = CONTROLLERS.create('neupan', config, limits)
    except (ImportError, ValueError) as error:
        print(f'bench_neupan: {error}')
        return 2
    import torch

    print(
        f'NeuPAN on {os.cpu_count()} cores, torch threads {torch.get_num_threads()}, '
        f'{config.dune_points} DUNE points, {config.horizon_steps} steps of {config.step_sec} s'
    )
    worst = 0.0
    for name, obstacles in SCENES.items():
        ms = np.array(walk(controller, obstacles, period, speed, args.solves + WARMUP)) * 1000.0
        ms = ms[WARMUP:]
        slow = float(np.percentile(ms, 95))
        worst = max(worst, slow)
        print(
            f'{name:<14} {len(ms)} solves: median {np.median(ms):.1f} ms, 95 % {slow:.1f} ms, '
            f'max {ms.max():.1f} ms'
        )
    return verdict(worst, period * 1000.0, config.step_sec * 1000.0)


def verdict(slow_ms: float, tick_ms: float, step_ms: float) -> int:
    """Print what the slow solves mean for the follower; return the exit code."""
    if slow_ms <= tick_ms:
        print(f"fits: a tick that solves stays within the follower's {tick_ms:.0f} ms tick")
        return 0
    if slow_ms <= step_ms:
        print(
            f'fits: within the {step_ms:.0f} ms step; a tick that solves is longer than the '
            f'{tick_ms:.0f} ms tick, the next one comes late'
        )
        return 0
    print(
        f'too slow: longer than the {step_ms:.0f} ms step, the commands come late: try '
        '--threads 2 or 4 (then neupan/threads), lower neupan/dune_points or horizon_steps'
    )
    return 1


if __name__ == '__main__':
    sys.exit(main())
