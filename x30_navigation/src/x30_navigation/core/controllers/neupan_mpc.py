"""NeuPAN: model predictive control straight from the obstacle points, heading included."""

# The route ahead (at the path shift offset) is NeuPAN's reference path and the safety scan's
# points are its obstacles. Each solve plans forward and sideways speed and the yaw rate over the
# horizon within the robot's speed and acceleration limits, keeping the footprint off every point
# through DUNE, the network trained for the footprint (models/, train_dune). This is the
# 'holonomic' kinematics of NeuPAN patched for the X30 (tools/neupan); a robot that cannot strafe
# uses NeuPAN's 'diff'.
#
# NeuPAN is an optional dependency (tools/install_neupan.sh): this module imports it only when the
# controller is built, and the build fails with the reason when NeuPAN or a model for the
# footprint is missing. The controller keeps its own clearance, so reactive avoidance stands by;
# path shift still picks the side to pass an obstacle on (NeuPAN alone stands before one right on
# the route, with no side to prefer). The guard and the control mux's protective field stay.

from __future__ import annotations

from dataclasses import dataclass
import math
import time
import warnings

import numpy as np

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Command,
    ControllerLimits,
    PathController,
    TrackRequest,
)
from x30_navigation.core.controllers.neupan_inputs import map_points, route_ahead
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
    weight_speed: float = param(
        2.0,
        min=0.0,
        max=100.0,
        live=False,
        doc='speed error cost: below 2 it stands before obstacles',
    )
    weight_lateral: float = param(
        1.0, min=0.0, max=100.0, live=False, doc='cost of walking sideways (0: free)'
    )
    clearance_gain: float = param(
        10.0, min=0.0, max=1000.0, live=False, doc='reward per m of clearance up to max_clearance'
    )
    max_clearance: float = param(
        0.25,
        min=0.0,
        max=5.0,
        unit='m',
        live=False,
        doc='clearance rewarded up to here: passages narrower than the body plus twice it stall',
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
            from neupan.robot import robot
            import torch
        except ImportError as error:
            raise ImportError(
                f'controller neupan needs NeuPAN ({error}): run tools/install_neupan.sh'
            ) from error
        if not hasattr(robot, 'linear_holonomic_model'):
            raise ImportError(
                'NeuPAN without the X30 patches (no holonomic kinematics): '
                'run tools/install_neupan.sh'
            )
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
        with warnings.catch_warnings():  # NeuPAN loads our own model, weights only
            warnings.filterwarnings('ignore', message='You are using `torch.load`')
            return self._neupan(neupan, kinematics, max_speed, max_acce)

    def _neupan(self, neupan, kinematics: str, max_speed: list, max_acce: list):
        """Return NeuPAN built with this controller's parameters."""
        cfg, lim = self.config, self.limits
        return neupan(
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
        cfg = self.config
        points = map_points(request, cfg.obstacle_range)
        path = route_ahead(request, cfg.window, cfg.spacing, cfg.heading_lookahead)
        self.planner.set_initial_path(path)
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
