"""NeuPAN: model predictive control straight from the obstacle points, heading included.

The route ahead (at the path shift offset) is NeuPAN's reference path and the safety scan's
points are its obstacles. Each solve plans forward and sideways speed and the yaw rate over the
horizon within the robot's speed and acceleration limits, keeping the footprint off every point
through DUNE, the network trained for the footprint (models/, train_dune). This is the
'holonomic' kinematics of NeuPAN patched for the X30 (third_party/neupan); a robot that cannot
strafe uses NeuPAN's 'diff'.

NeuPAN is an optional dependency (tools/install_neupan.sh): this module imports it only when the
controller is built, and the build fails with the reason when NeuPAN or a model for the
footprint is missing. The controller avoids by itself, so the follower's path shift and reactive
avoidance stand by; the guard and the control mux's protective field stay as they are.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    ControllerLimits,
    PathController,
    TrackRequest,
)
from x30_navigation.core.dune_model import DuneRecord, check_checkpoint
from x30_utils.config import param


@dataclass(frozen=True)
class NeupanConfig:
    """Parameters of the NeuPAN controller (read at start: the solver is built once)."""

    dune_checkpoint: str = param(
        '', live=False, doc='DUNE model of the footprint (.pth, its .json beside it)'
    )
    horizon_steps: int = param(10, min=2, max=50, live=False, doc='MPC steps ahead')
    step_sec: float = param(
        0.1, min=0.02, max=0.5, unit='s', live=False, doc='MPC step, also the time between solves'
    )
    iterations: int = param(2, min=1, max=10, live=False, doc='PAN iterations per solve')
    dune_points: int = param(
        100, min=1, max=2000, live=False, doc='obstacle points through DUNE (decimated beyond)'
    )
    nrmp_points: int = param(10, min=1, max=100, live=False, doc='nearest points in the MPC')
    obstacle_range: float = param(
        4.0,
        min=0.5,
        max=50.0,
        unit='m',
        live=False,
        doc="points further from base_link are left out (inside the model's training range)",
    )
    window: float = param(5.0, min=0.5, max=50.0, unit='m', live=False, doc='route ahead given')
    spacing: float = param(
        0.1, min=0.02, max=1.0, unit='m', live=False, doc='between the route points given'
    )
    heading_lookahead: float = param(
        0.6, min=0.0, max=5.0, unit='m', live=False, doc='the heading follows the route this far'
    )
    weight_xy: float = param(1.0, min=0.0, max=100.0, live=False, doc='position error cost')
    weight_yaw: float = param(0.5, min=0.0, max=100.0, live=False, doc='heading error cost')
    weight_speed: float = param(1.0, min=0.0, max=100.0, live=False, doc='speed error cost')
    weight_lateral: float = param(
        1.0, min=0.0, max=100.0, live=False, doc='cost of walking sideways (0: free)'
    )
    clearance_gain: float = param(
        15.0, min=0.0, max=1000.0, live=False, doc='reward per m of clearance up to max_clearance'
    )
    max_clearance: float = param(
        1.0, min=0.0, max=5.0, unit='m', live=False, doc='clearance rewarded up to here'
    )
    min_clearance: float = param(
        0.1, min=0.0, max=2.0, unit='m', live=False, doc='clearance kept from every point'
    )
    stop_distance: float = param(
        0.05, min=0.0, max=1.0, unit='m', live=False, doc='a point this near the body: stand'
    )
    max_reverse_speed: float = param(
        0.15, min=0.0, max=5.0, unit='m/s', live=False, doc='backward speed cap'
    )
    max_failures: int = param(
        10, min=1, max=1000, live=False, doc='failed solves in a row before giving up (stop)'
    )
    threads: int = param(1, min=0, max=64, live=False, doc='torch threads (0: torch default)')

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.min_clearance > self.max_clearance:
            problems.append('min_clearance must not exceed max_clearance')
        if self.nrmp_points > self.dune_points:
            problems.append('nrmp_points must not exceed dune_points')
        return problems


def body_vertices(limits: ControllerLimits) -> np.ndarray:
    """Return the footprint corners as NeuPAN vertices, (2, N)."""
    return np.asarray(limits.footprint, dtype=float).reshape(-1, 2).T


@CONTROLLERS.register('neupan')
class NeupanController(PathController):
    """Plan (vx, vy, wz) over a horizon with NeuPAN: route tracking and point avoidance at once."""

    Config = NeupanConfig
    strafes = True
    avoids = True

    def __init__(self, config=None, limits=None):
        super().__init__(config, limits)
        cfg = self.config
        problems = check_checkpoint(cfg.dune_checkpoint, body_vertices(self.limits))
        if problems:
            raise ValueError('; '.join(problems))
        trained = DuneRecord.load(cfg.dune_checkpoint).range_m
        travel = self.limits.max_vx * cfg.horizon_steps * cfg.step_sec
        if trained > 0.0 and cfg.obstacle_range + travel > trained:
            # DUNE sees the points from every planned pose: keep them where it was trained
            raise ValueError(
                f"obstacle_range {cfg.obstacle_range:.2f} m and the horizon's {travel:.2f} m "
                f'reach beyond the {trained:.2f} m the DUNE model was trained for'
            )
        self.holonomic = self.limits.max_vy > 0.0
        self.planner = self._build()
        self.since_solve = math.inf
        self.plan = np.zeros(3)
        self.failures = 0

    def _build(self):
        """Return NeuPAN for the robot (ImportError: NeuPAN is not installed)."""
        try:
            from neupan import neupan
            import torch
        except ImportError as error:
            raise ImportError(
                f'controller neupan needs NeuPAN ({error}): run tools/install_neupan.sh'
            ) from error
        cfg, lim = self.config, self.limits
        if cfg.threads > 0:
            torch.set_num_threads(cfg.threads)
        if self.holonomic:
            kinematics = 'holonomic'
            max_speed = [lim.max_vx, lim.max_vy, lim.max_wz]
            max_acce = [lim.max_accel, lim.max_accel, lim.max_yaw_accel]
        else:
            kinematics = 'diff'
            max_speed = [lim.max_vx, lim.max_wz]
            max_acce = [lim.max_accel, lim.max_yaw_accel]
        planner = neupan(
            receding=cfg.horizon_steps,
            step_time=cfg.step_sec,
            ref_speed=max(lim.max_vx, 0.01),
            device='cpu',
            collision_threshold=cfg.stop_distance,
            robot_kwargs={
                'kinematics': kinematics,
                'vertices': body_vertices(lim).T.tolist(),
                'max_speed': max_speed,
                'max_acce': max_acce,
                'lateral_weight': cfg.weight_lateral,
                'name': 'x30_navigation',
            },
            ipath_kwargs={
                # the follower decides the arrival: NeuPAN only near the very end
                'arrive_threshold': 0.01,
                'close_threshold': 0.05,
                'ind_range': 20,
                'arrive_index_threshold': 1,
            },
            pan_kwargs={
                'iter_num': cfg.iterations,
                'dune_max_num': cfg.dune_points,
                'nrmp_max_num': cfg.nrmp_points,
                'iter_threshold': 0.1,
                'dune_checkpoint': cfg.dune_checkpoint,
            },
            adjust_kwargs={
                'q_s': [cfg.weight_xy, cfg.weight_xy, cfg.weight_yaw],
                'p_u': cfg.weight_speed,
                'eta': cfg.clearance_gain,
                'd_max': cfg.max_clearance,
                'd_min': cfg.min_clearance,
            },
            train_kwargs={},
        )
        if self.holonomic and getattr(planner.robot, 'control_dim', 2) != 3:
            raise ImportError(
                'NeuPAN has no holonomic kinematics: install the patched one '
                '(tools/install_neupan.sh)'
            )
        return planner

    def reset(self):
        """Forget the plan and NeuPAN's warm start (new route, restart)."""
        super().reset()
        self.planner.reset()
        self.since_solve = math.inf
        self.plan = np.zeros(3)
        self.failures = 0

    def compute(self, request: TrackRequest) -> Command:
        """Return the planned command: a new solve every step_sec, the last one in between."""
        self.since_solve += max(request.dt, 0.0)
        if self.since_solve >= self.config.step_sec - 1e-9:
            self.since_solve = 0.0
            self.plan = self._solve(request)
        return self._command(self.plan)

    def _solve(self, request: TrackRequest) -> np.ndarray:
        """Return (vx, vy, wz) of a new solve; zero after a failed one."""
        if request.route.length < self.config.spacing:
            return np.zeros(3)
        pose = request.pose
        state = np.array([[pose.x], [pose.y], [pose.yaw]])
        points = self.world_points(request)
        self.planner.set_initial_path(self.reference(request))
        self.planner.set_reference_speed(max(request.speed, 0.0))
        start = time.perf_counter()
        try:
            action, info = self.planner(state, points)
        except Exception as error:  # a solver error: stand, try again next step
            self.failures += 1
            self.debug = {'error': str(error)}
            self.planner.reset()
            if self.failures >= self.config.max_failures:
                raise RuntimeError(f'NeuPAN failed {self.failures} times: {error}') from error
            return np.zeros(3)
        self.failures = 0
        action = np.asarray(action, dtype=float).reshape(-1)
        if not self.holonomic:
            action = np.array([action[0], 0.0, action[1]])
        self.debug = self._debug(request, info, time.perf_counter() - start, points)
        if info.get('stop') or info.get('arrive') or not np.all(np.isfinite(action)):
            return np.zeros(3)
        return action

    def _command(self, plan: np.ndarray) -> Command:
        """Return the plan as a command within the robot's limits."""
        lim = self.limits
        vx = float(np.clip(plan[0], -self.config.max_reverse_speed, lim.max_vx))
        vy = float(np.clip(plan[1], -lim.max_vy, lim.max_vy))
        return Command(vx, vy, self.clamp_wz(float(plan[2])))

    def reference(self, request: TrackRequest) -> list:
        """Return the route ahead as NeuPAN path points [x, y, yaw, gear=1] (4, 1)."""
        cfg, route = self.config, request.route
        end = route.length
        start = min(request.s, max(0.0, end - cfg.spacing))
        stop = min(end, start + max(cfg.window, cfg.spacing))
        steps = np.append(np.arange(start, stop - 1e-6, cfg.spacing), stop)
        path = []
        for s in steps:
            x, y = route.shifted_point(float(s), request.offset)
            if cfg.heading_lookahead > 0.0 and s < end:
                yaw = route.chord_heading(float(s), min(end, float(s) + cfg.heading_lookahead))
            else:
                yaw = float(route.segment_yaw[route.segment_at(float(s))])
            path.append(np.array([[x], [y], [yaw], [1.0]]))
        return path

    def world_points(self, request: TrackRequest):
        """Return the obstacle points within range in the map frame (2, N), or None."""
        points = np.asarray(request.obstacles, dtype=float).reshape(-1, 2)
        if points.size == 0:
            return None
        points = points[np.hypot(points[:, 0], points[:, 1]) <= self.config.obstacle_range]
        if points.size == 0:
            return None
        pose = request.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        world = points @ np.array([[c, s], [-s, c]]) + (pose.x, pose.y)
        return world.T

    def _debug(self, request: TrackRequest, info: dict, seconds: float, points) -> dict:
        """Return what the markers draw: the planned poses, the route ahead, the timing."""
        planned = info.get('opt_state_list') or []
        rollout = np.array([[float(p[0, 0]), float(p[1, 0])] for p in planned])
        reference = info.get('ref_state_list') or []
        out = {
            'rollout': rollout,
            'nearest': tuple(request.route.shifted_point(request.s, request.offset)),
            'solve_ms': round(seconds * 1000.0, 1),
            'points': 0 if points is None else int(points.shape[1]),
            'min_distance': float(self.planner.min_distance),
            'stop': bool(info.get('stop')),
        }
        if reference:
            out['carrot'] = (float(reference[-1][0, 0]), float(reference[-1][1, 0]))
        return out
