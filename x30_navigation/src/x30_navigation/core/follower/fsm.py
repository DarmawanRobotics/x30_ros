"""The path follower as a pure state machine: Follower.tick(inputs) -> Decision."""

from __future__ import annotations

import math

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers.base import Command, PathController
from x30_navigation.core.follower.approach import FinalApproach
from x30_navigation.core.follower.backtrack import Backtrack, BacktrackSteps
from x30_navigation.core.follower.command import CommandShaper
from x30_navigation.core.follower.detour import Detour, DetourSteps
from x30_navigation.core.follower.guard import MotionGuard
from x30_navigation.core.follower.pause import PauseReasons
from x30_navigation.core.follower.phases import Phases
from x30_navigation.core.follower.reverse import Reverse, ReverseSteps
from x30_navigation.core.follower.state import (
    DONE,
    Decision,
    FollowerState,
    Inputs,
    Report,
    StopEventKind,
    check_transition,
)
from x30_navigation.core.follower.stops import Inspection, StopPoints
from x30_navigation.core.follower.supervisor import Problem, Supervisor
from x30_navigation.core.follower.tracker import Tracker
from x30_navigation.core.path.progress import RouteProgress
from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile

S = FollowerState


class Follower(Phases, ReverseSteps, DetourSteps, BacktrackSteps):
    """Follow a route: join it, follow it, stop at its stop points, reach its end."""

    def __init__(self, config: FollowerConfig, robot: RobotProfile, controller: PathController):
        self.config, self.robot = config, robot
        self.controller, self.controller_name = controller, config.controller
        self.pauses = PauseReasons()
        self.supervisor = Supervisor(config)
        self.shaper = CommandShaper(robot, config.speed.smoothing)
        self.guard = MotionGuard(robot, config.safety)
        self.approach = FinalApproach(config.approach, robot.can_strafe)
        self.inspection = Inspection()
        self.tracker: Tracker | None = None
        self.stops: StopPoints | None = None
        self.state = self.phase = S.INACTIVE
        self.target, self.goal, self.timer, self.lap = 0, False, 0.0, 0
        self.zero_pending = False
        self.forward_held = False  # this tick: an emergency holds the forward motion only
        self.reverse, self.detour, self.block_note = Reverse(), Detour(), ''
        self.backtrack = Backtrack()
        self.message = 'no route: call navigation/set_path or navigation/load_path'
        self.events, self.debug, self.inspect = [], {}, None

    @property
    def active(self) -> bool:
        """True while a run is going on (moving, paused or blocked)."""
        return self.phase not in DONE and self.phase is not S.INACTIVE

    def set_route(self, route: Route, pose: Pose2D | None = None) -> tuple[bool, str]:
        """Take a new route; a running follower starts over on it from where the robot is."""
        was_active = self.active
        self.tracker = Tracker(route, self.robot, self.config, self.controller)
        self.stops = StopPoints(route, self.config.approach)
        self.lap = 0
        self.inspection.cancel()
        if not was_active:
            self.message = f'route of {len(route)} points, {len(route.stops)} stop points'
            return True, self.message
        ok, message = self.start(pose, resume=False)
        if not ok:
            self.stop(message)
        return ok, message

    def set_controller(self, name: str, controller: PathController):
        """Switch the path controller, keeping the progress."""
        self.controller, self.controller_name = controller, name
        if self.tracker is not None:
            self.tracker.controller = controller
        controller.reset()

    def configure(self, config: FollowerConfig):
        """Apply new parameters, keeping the route and the progress."""
        self.config = config
        self.supervisor.configure(config)
        self.approach = FinalApproach(config.approach, self.robot.can_strafe)
        if self.tracker is not None:
            s = self.tracker.progress.s
            self.tracker = Tracker(self.tracker.route, self.robot, config, self.controller)
            self.tracker.progress.reset(s)
            self.stops.config = config.approach

    def start(self, pose: Pose2D | None, resume: bool = True) -> tuple[bool, str]:
        """Start on the route from where the robot is (resume: keep a nearby progress)."""
        if self.tracker is None:
            return False, 'no route: call navigation/set_path or navigation/load_path first'
        if pose is None:
            return False, 'robot pose unknown: is the robot localized?'
        blocking = [r for r in self.pauses.blocking_start() if r in ('estop', 'localization')]
        if blocking:
            return False, f'cannot start while {", ".join(blocking)} holds'
        self.pauses.clear_operator()
        ok, message = self._locate(pose, resume)
        if ok:
            self.supervisor.reset()
            self.zero_pending = False
        return ok, message

    def stop(self, message: str = 'stopped') -> Command:
        """End the run; returns the zero command to publish now."""
        self.inspection.cancel()
        self.reverse.end()
        self.backtrack.reset()
        self._finish(S.INACTIVE, message)
        self.zero_pending = False
        return self.shaper.stop()

    def pause(self):
        """Hold position, keep the progress (operator pause)."""
        self.pauses.pause()

    def resume(self) -> tuple[bool, str]:
        """Clear the operator pause and a released e-stop."""
        return self.pauses.resume()

    def on_control(self, estop: bool, remote: bool, armed: bool = True):
        """Apply control/status (armed: navigation may drive; neutral commands arm it)."""
        self.pauses.on_control(estop, remote, armed)

    def on_localization(self, localized: bool):
        """Apply localization/status."""
        self.pauses.on_localization(localized, self.config.require_localization)

    def on_battery(self, fraction: float):
        """Apply a battery reading (0..1; others ignored)."""
        self.pauses.on_battery(self.supervisor.battery(fraction)[1])

    def inspection_started(self, round_id: int, count: int, unavailable: list[str]):
        """Record the inspection calls the node made; returns events to publish."""
        if self.inspection.started(round_id, count, unavailable):
            self._event(StopEventKind.INSPECTED, self.target, self.inspection.summary())
        return self._take_events()

    def inspection_answer(self, round_id: int, name: str, ok: bool, message: str):
        """Record an inspection answer; returns events to publish."""
        if self.inspection.answer(round_id, name, ok, message):
            self._event(StopEventKind.INSPECTED, self.target, self.inspection.summary())
        return self._take_events()

    def tick(self, inputs: Inputs) -> Decision:
        """Run one control tick."""
        self.debug, self.inspect, self.forward_held = {}, None, False
        if not self.active:
            command = self.shaper.stop() if self.zero_pending else None
            self.zero_pending = False
            return self._decision(command, self.phase, self.message)
        if self.pauses.active:
            self.backtrack.watch.reset()
            return self._hold(S.PAUSED, f'paused: {", ".join(self.pauses.names())}', False)
        check = self.supervisor.pose(inputs)
        problem = check.problem or self.supervisor.scan(inputs)
        if problem is None:
            self._watch_stuck(inputs, check.pose)
        # avoidance holds neither a standing (dwelling) nor a backing-up robot
        backing = self.reverse.active or self.backtrack.active
        if problem is None and self.phase is not S.DWELLING and not backing:
            required = self.config.avoidance_mode != 'off'
            problem = self.supervisor.avoidance(inputs, required)
            if problem is Problem.EMERGENCY and self._slides_on_emergency():
                problem, self.forward_held = None, True
        if problem is not None:
            counts = problem in (Problem.EMERGENCY, Problem.SCAN_BLIND)
            message = f'{problem.name}: {problem.value}'
            return self._hold(None, message, counts, inputs.dt, check.pose)
        return self._move(inputs, check.pose)

    def _move(self, inputs: Inputs, pose: Pose2D) -> Decision:
        step = {
            S.FOLLOWING: self._following,
            S.APPROACHING: self._approaching,
            S.JOINING: self._joining,
            S.DWELLING: self._dwelling,
        }[self.phase]
        target, message = step(inputs, pose)
        if not self.active:
            self.zero_pending = False
            return self._decision(self.shaper.stop(), self.phase, message, pose)
        guarded = self._guard(target or Command(), inputs, pose)
        if self.forward_held:
            guarded = self._hold_forward(guarded.command, inputs)
            message = (
                f'{Problem.EMERGENCY.name}: {Problem.EMERGENCY.value}, forward held | {message}'
            )
        smooth = self.phase is S.FOLLOWING
        shaped = self.shaper.shape(guarded.command, inputs.dt, smooth)
        scan = inputs.scan  # what is published is guarded too: the ramps are only comfort
        command = self.guard.published(shaped, guarded.command, scan.guard_points, scan.blocked)
        if command != shaped:
            self.shaper.hold(command)
        if guarded.stalled:
            message = f'{message} | waiting for an obstacle {guarded.clearance:.2f} m ahead'
        counts = self.phase is not S.DWELLING
        blocked = self.supervisor.blocked(pose, counts, inputs.dt)
        if blocked and self.block_note:
            message = f'{self.block_note} | {message}'
        elif not blocked:
            self.block_note = ''
        self.debug['pose'] = pose
        return self._decision(command, S.BLOCKED if blocked else self.phase, message, pose)

    def _guard(self, target: Command, inputs: Inputs, pose: Pose2D):
        """Guard the target; a held command still slides toward its offset, or goes on parallel."""
        scan = inputs.scan
        guarded = self.guard.limit(target, scan.guard_points, scan.blocked)
        if not guarded.stalled or self.phase is not S.FOLLOWING or self.detour.active:
            return guarded
        tracker = self.tracker
        for held in (
            tracker.held_slide(pose, inputs.dt, scan),
            tracker.held_parallel(pose, target),
        ):
            if held is not None:
                moving = self.guard.limit(held, scan.guard_points, scan.blocked)
                if not moving.stalled:
                    return moving
        return guarded

    def _hold(self, state, message: str, counts: bool, dt: float = 0.0, pose=None) -> Decision:
        """Publish zero this tick; held by a problem the standing time counts (BLOCKED)."""
        self.detour.end()
        blocked = self.supervisor.blocked(pose, counts, dt)
        if state is None:
            state = S.BLOCKED if blocked else self.phase
        return self._decision(self.shaper.stop(), state, message)

    def _decision(self, command, state: FollowerState, message: str, pose=None) -> Decision:
        if state is not self.state:
            check_transition(self.state, state)
            self.state = state
        self.message = message
        return Decision(command, self._report(pose), self._take_events(), self.inspect, self.debug)

    def _report(self, pose: Pose2D | None) -> Report:
        reasons = self.pauses.names()
        if self.tracker is None:
            return Report(self.state, self.message, reasons)
        progress = self.tracker.progress
        lateral = self.tracker.lateral(pose) if pose is not None else 0.0
        count = len(self.tracker.route)
        return Report(
            self.state, self.message, reasons, progress.index, count, progress.remaining, lateral
        )

    def _locate(self, pose: Pose2D, resume: bool) -> tuple[bool, str]:
        tracker, cfg = self.tracker, self.config
        if resume and self.active and abs(tracker.lateral(pose)) <= cfg.safety.off_path_slow:
            return True, 'resumed where it was'
        s = RouteProgress(tracker.route).locate(pose)
        foot = tracker.route.point_at(s)
        distance = math.hypot(foot[0] - pose.x, foot[1] - pose.y)
        reach = max(cfg.join.max_distance, cfg.safety.off_path_slow)
        if distance > reach:
            return False, f'robot is {distance:.1f} m from the route (limit {reach:.1f} m)'
        self._restart(s)
        if distance <= cfg.safety.off_path_slow:
            return True, f'on the route at point {tracker.progress.index}'
        self.target, self.timer = tracker.route.index_at(s), 0.0
        self.approach.reset()
        self._set_phase(S.JOINING)
        return True, f'joining the route at point {self.target}, {distance:.1f} m away'

    def _restart(self, s: float):
        self.reverse.end()
        self.detour.end()
        self.backtrack.reset()
        self.tracker.restart(s)
        self.stops.reset(s)
        self.inspection.cancel()
        self.supervisor.reset()
        self._set_phase(S.FOLLOWING)

    def _set_phase(self, phase: FollowerState):
        self.phase = phase

    def _finish(self, state: FollowerState, message: str = ''):
        self.phase = state
        self.detour.end()
        self.zero_pending = True
        if message:
            self.message = message

    def _take_events(self):
        events, self.events = self.events, []
        return events
