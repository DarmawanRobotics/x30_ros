"""Detours: turn in place toward a gap and drive through it when sliding does not get on."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

from x30_navigation.core.config.shift import DetourConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.follower.state import Inputs
from x30_navigation.core.separation import protective_distance
from x30_navigation.core.shift.corridor import Corridor
from x30_utils.math import Pose2D, angle_diff
from x30_utils.safety import rotation_room

ANGLE_STEP_DEG = 5.0
TIE_M = 0.05  # runs this close in progress: the smaller turn wins
NO_TURN = math.radians(2.0)  # a smaller turn needs no rotation room
TURN_DONE = math.radians(3.0)  # TURN ends within this of the heading
KP_YAW = 1.5  # 1/s
MAX_TURN_RATE = 0.6  # rad/s
MOVED_M, MOVED_RAD = 0.004, 0.004  # per tick: the robot moves
HELD_SEC = 2.0  # held this long by the guard (or an emergency): the detour ends
DONE_M = 0.03
STUCK_SPEED = 0.1  # m/s: slower on average over after_sec (and turned less): stuck, creeping too
STUCK_RAD = math.radians(15.0)


@dataclass(frozen=True)
class DetourPlan:
    """One straight run: its heading (map frame), length and progress along the route."""

    heading: float
    length: float
    progress: float
    turn: float  # rad, from the robot's yaw when planned
    angle: float  # rad, from the route direction


@dataclass
class Detour:
    """The run going on, if any, and the time standing since the last one."""

    plan: DetourPlan | None = None
    start: Pose2D | None = None
    last: Pose2D | None = None
    turning: bool = False
    seconds: float = 0.0
    held: float = 0.0
    trail: deque = field(default_factory=deque)  # (time, pose) while following, after_sec

    @property
    def active(self) -> bool:
        """True while turning or driving a run."""
        return self.plan is not None

    def begin(self, plan: DetourPlan, pose: Pose2D):
        """Start the run of plan from pose."""
        self.plan, self.start, self.last = plan, pose, pose
        self.turning, self.seconds, self.held = True, 0.0, 0.0
        self.trail.clear()

    def end(self):
        """Stop the run (following goes on); the next one only after standing again."""
        self.plan = self.start = self.last = None
        self.turning = False
        self.trail.clear()

    def stuck(self, now: float, pose: Pose2D, seconds: float) -> bool:
        """Return True when the robot was slower than STUCK_SPEED (turned less) for seconds."""
        trail = self.trail
        trail.append((now, pose))
        while len(trail) > 1 and now - trail[1][0] >= seconds:
            trail.popleft()
        then, old = trail[0]
        if now - then < seconds - 1e-9:
            return False
        moved = math.hypot(pose.x - old.x, pose.y - old.y)
        return moved < STUCK_SPEED * seconds and abs(angle_diff(pose.yaw, old.yaw)) < STUCK_RAD

    def travelled(self, pose: Pose2D) -> float:
        """Return how far the robot moved along the run's heading (m)."""
        dx, dy = pose.x - self.start.x, pose.y - self.start.y
        return dx * math.cos(self.plan.heading) + dy * math.sin(self.plan.heading)

    def moved(self, pose: Pose2D) -> bool:
        """Return True when the robot moved or turned since the last tick (and remember it)."""
        last, self.last = self.last, pose
        step = math.hypot(pose.x - last.x, pose.y - last.y)
        return step > MOVED_M or abs(angle_diff(pose.yaw, last.yaw)) > MOVED_RAD


class DetourPlanner:
    """Choose the straight run that takes the robot furthest along the route."""

    def __init__(self, guard: MotionGuard, yaw_margin: float):
        self.guard = guard
        self.footprint = guard.robot.footprint
        self.yaw_margin = yaw_margin

    def plan(
        self,
        config: DetourConfig,
        corridor: Corridor,
        route_yaw: float,
        s: float,
        lateral: float,
        pose: Pose2D,
        points: np.ndarray,
        limit: float,
    ) -> DetourPlan | None:
        """Return the best run from pose, or None."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        limits, safety = self.guard.robot.limits, self.guard.config
        stop = protective_distance(
            config.speed, limits.max_decel, limits.reaction_time, safety.human_speed
        )
        best = None
        angles = np.arange(-config.max_angle_deg, config.max_angle_deg + 1e-9, ANGLE_STEP_DEG)
        for angle in np.radians(angles):
            run = (route_yaw, s, lateral, pose, points, limit, float(angle), stop)
            plan = self._run(config, corridor, *run)
            if plan is not None and _better(plan, best):
                best = plan
        return best

    def _run(
        self,
        config: DetourConfig,
        corridor: Corridor,
        route_yaw: float,
        s: float,
        lateral: float,
        pose: Pose2D,
        points: np.ndarray,
        limit: float,
        angle: float,
        stop: float,
    ) -> DetourPlan | None:
        """Return the run at angle off the route direction, or None when it does not qualify."""
        safety = self.guard.config
        heading = route_yaw + angle
        turn = angle_diff(heading, pose.yaw)
        if not self._can_turn(points, turn):
            return None
        c, s_ = math.cos(turn), math.sin(turn)
        body, band = self.guard.free(1.0, 0.0, points @ np.array([[c, -s_], [s_, c]]))
        free = min(body - safety.stop_margin, band - safety.pass_margin) - stop
        length = min(free, config.length, self._side_room(angle, lateral, limit))
        if length <= DONE_M:
            return None
        end = np.array(
            [[pose.x + length * math.cos(heading), pose.y + length * math.sin(heading)]]
        )
        s_end, d_end = corridor.coordinates(s, end)
        if not len(s_end) or abs(float(d_end[0])) > limit + 1e-6:
            return None
        progress = float(s_end[0]) - s
        if progress < config.min_progress:
            return None
        return DetourPlan(heading, length, progress, turn, angle)

    def _can_turn(self, points: np.ndarray, turn: float) -> bool:
        if abs(turn) < NO_TURN or not len(points):
            return True
        return rotation_room(self.footprint, points, turn) - self.yaw_margin >= abs(turn)

    @staticmethod
    def _side_room(angle: float, lateral: float, limit: float) -> float:
        """Return how far a run at angle off the route goes before it is limit from the route."""
        side = math.sin(angle)
        if abs(side) < 1e-6:
            return math.inf
        return (limit - lateral) / side if side > 0.0 else (limit + lateral) / -side


class DetourSteps:
    """Follower steps of a detour (a mixin of Follower, like Phases)."""

    def _detour_step(self, inputs: Inputs, pose: Pose2D, s: float, lateral: float, hold: bool):
        """Run or start a detour; return (command, message) while one goes on, else None."""
        cfg, detour, tracker = self.config.detour, self.detour, self.tracker
        if not detour.active:
            if hold or not cfg.enabled:  # near a stop point the robot waits on the route
                detour.trail.clear()
                return None
            if not detour.stuck(inputs.now, pose, cfg.after_sec):
                return None
            detour.trail.clear()  # planned again only after_sec later
            shift = self.config.path_shift
            planner = DetourPlanner(self.guard, math.radians(self.config.safety.yaw_margin_deg))
            route = tracker.route
            plan = planner.plan(
                cfg,
                tracker.shift.corridor,
                float(route.segment_yaw[route.segment_at(s)]),
                s,
                lateral,
                pose,
                inputs.scan.guard_points,
                max(shift.max_offset, shift.search_offset),
            )
            if plan is None:
                return None
            detour.begin(plan, pose)
        return self._detouring(inputs, pose, s, lateral)

    def _detouring(self, inputs: Inputs, pose: Pose2D, s: float, lateral: float):
        """Turn onto the run's heading, then drive it; None once it ended."""
        detour, plan, shift = self.detour, self.detour.plan, self.tracker.shift
        shift.offset = shift.target = lateral  # following goes on from here (no off-path)
        shift.return_since = None
        detour.seconds += inputs.dt
        detour.held = 0.0 if detour.moved(pose) else detour.held + inputs.dt
        travelled = detour.travelled(pose)
        timeout = 2.0 * (abs(plan.turn) / MAX_TURN_RATE + plan.length / self.config.detour.speed)
        done = travelled >= plan.length - DONE_M or detour.seconds > timeout + 5.0
        if done or detour.held > HELD_SEC or self._way_open(inputs, pose, s, lateral):
            detour.end()
            shift.searching = abs(lateral) > self.config.path_shift.max_offset + 1e-9
            return None
        error = angle_diff(plan.heading, pose.yaw)
        wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, KP_YAW * error))
        angle = f'{math.degrees(plan.angle):+.0f} deg'
        if detour.turning and abs(error) > TURN_DONE:
            return Command(0.0, 0.0, wz), f'detour: turning to {angle} off the route, toward a gap'
        detour.turning = False
        message = f'detour: {plan.length - travelled:.1f} m on at {angle} off the route'
        return Command(self.config.detour.speed, 0.0, wz), message

    def _way_open(self, inputs: Inputs, pose: Pose2D, s: float, lateral: float) -> bool:
        """Return True when nothing is ahead along the route on the robot's line any more."""
        scan, corridor = inputs.scan, self.tracker.shift.corridor
        points = pose.apply(scan.points) if len(scan.points) else np.zeros((0, 2))
        s_pts, d_pts = corridor.coordinates(s, points)
        free = corridor.free_distances(s_pts, d_pts, s, [lateral], passing=True)
        return not scan.blocked and bool(np.isinf(free[0]))


def _better(plan: DetourPlan, best: DetourPlan | None) -> bool:
    """Return True when plan goes further along the route, or as far with a smaller turn."""
    if best is None or plan.progress > best.progress + TIE_M:
        return True
    return plan.progress > best.progress - TIE_M and abs(plan.turn) < abs(best.turn)
