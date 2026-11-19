"""The moving phases of the follower: following, joining, final approach and dwelling."""

from __future__ import annotations

import math

from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.state import FollowerState, Inputs, StopEvent
from x30_navigation.core.follower.state import StopEventKind as Ev
from x30_navigation.core.follower.stops import StopCheck
from x30_utils.math import Pose2D

S = FollowerState
JOIN_YAW = math.radians(20.0)


class Phases:
    """Phase steps of Follower; each returns (target command or None, message)."""

    def _following(self, inputs: Inputs, pose: Pose2D):
        tracker, route = self.tracker, self.tracker.route
        s = tracker.advance(pose, inputs.dt)
        lateral = tracker.lateral(pose)
        slow, off = self.supervisor.off_path(max(0.0, abs(lateral - tracker.offset)))
        if off:
            self._finish(S.OFF_PATH)
            return None, f'{abs(lateral):.2f} m from the route: check the robot, start again'
        check = self.stops.check(s, pose, lateral)
        if check is StopCheck.SKIP:
            self._event(Ev.SKIPPED, self.stops.next, 'passed without reaching it')
            self.stops.advance()
        elif check is StopCheck.APPROACH:
            self._enter_approach(self.stops.next, goal=False)
            return Command(), f'stop point {self.target}'
        end = route.xy[-1]
        near = self.config.approach.radius
        if tracker.progress.remaining <= near and math.hypot(*(end - (pose.x, pose.y))) <= near:
            return self._route_end()
        window = near + self.robot.footprint.front + self.config.path_shift.look_ahead
        hold = self.stops.ahead(s) <= window
        detour = self._detour_step(inputs, pose, s, lateral, hold)
        if detour is not None:
            return detour
        forward, last = not self.forward_held, self.shaper.last  # emergency: slide only
        out = tracker.step(
            pose, inputs.scan, inputs.avoidance, inputs.now, inputs.dt, slow, hold, forward, last
        )
        self.debug.update(out.debug, offset=tracker.offset)
        search = ' searching' if tracker.shifting and tracker.shift.searching else ''
        offset = f'offset {tracker.offset:+.2f} m{search} | ' if tracker.shifting else ''
        solve = out.debug.get('solve_ms')  # an optimising controller's latest solve
        name = self.controller_name if solve is None else f'{self.controller_name} {solve:.0f} ms'
        message = (
            f'lap {self.lap} | {name} | {offset}speed {out.speed:.2f} m/s'
            f' | next stop {self.stops.next}'
        )
        return out.command, message

    def _slides_on_emergency(self) -> bool:
        """Return True when an emergency holds only the forward motion (path shift slides)."""
        tracker = self.tracker
        following = self.phase is S.FOLLOWING and tracker is not None and tracker.shifting
        return following and self.config.path_shift.emergency_slide

    def _hold_forward(self, command: Command, inputs: Inputs):
        """Stop the forward motion at once, keep the sideways slide and the turn, guarded again."""
        self.shaper.hold_forward()
        scan = inputs.scan
        return self.guard.limit(
            Command(0.0, command.vy, command.wz), scan.guard_points, scan.blocked
        )

    def _route_end(self):
        route = self.tracker.route
        closed = route.is_closed(self.config.loop_close_distance)
        if self.config.loop_route and closed and not self.supervisor.battery_low:
            self.lap += 1
            self._restart(0.0)
            return Command(), f'lap {self.lap} started'
        self._enter_approach(len(route) - 1, goal=True)
        return Command(), 'approaching the goal'

    def _approaching(self, inputs: Inputs, pose: Pose2D):
        self.timer += inputs.dt
        target = self.tracker.route.pose(self.target)
        step = self.approach.step(pose, target)
        timeout = self.timer > self.config.approach.timeout_sec
        if step.done or timeout:
            note = '' if step.done else f'approach timeout, {step.position_error:.2f} m off'
            if self.goal:
                self._finish(S.GOAL_REACHED)
                return None, note or 'at the goal'
            self._enter_dwell(note)
            return Command(), f'stop {self.target}: arrived {note}'.strip()
        self.debug['target'] = target
        name = 'goal' if self.goal else f'stop {self.target}'
        message = (
            f'{name}: {step.position_error * 100:.1f} cm, {math.degrees(step.yaw_error):.1f} deg'
        )
        return step.command, message

    def _joining(self, inputs: Inputs, pose: Pose2D):
        if self.backtrack.active:
            return self._backtracking(inputs, pose)
        if self.reverse.active:
            return self._reversing(inputs, pose)
        rejoining = self._rejoining(inputs, pose)
        if rejoining is not None:
            return rejoining
        self.timer += inputs.dt
        target = self.tracker.route.pose(self.target)
        step = self.approach.step(pose, target)
        on_route = (
            step.position_error < self.config.join.on_route and abs(step.yaw_error) < JOIN_YAW
        )
        if step.done or on_route or self.timer > self.config.join.timeout_sec:
            self.tracker.restart(float(self.tracker.route.s[self.target]))
            self._set_phase(S.FOLLOWING)
            return Command(), f'on the route at point {self.target}'
        self.debug['target'] = target
        message = (
            f'route point {self.target}: {step.position_error * 100:.0f} cm,'
            f' {math.degrees(step.yaw_error):.0f} deg'
        )
        return step.command, message

    def _dwelling(self, inputs: Inputs, pose: Pose2D):
        self.timer += inputs.dt
        dwell = float(self.tracker.route.dwell[self.target])
        timeout = dwell + self.config.hooks.inspection_timeout_sec
        waiting = self.inspection.pending > 0 and self.timer < timeout
        if self.timer < dwell or waiting:
            pending = self.inspection.pending
            note = f', {pending} inspection(s) pending' if pending else ''
            self.debug['target'] = self.tracker.route.pose(self.target)
            return Command(), f'resuming in {max(0.0, dwell - self.timer):.1f} s{note}'
        if self.inspection.pending:
            self._event(Ev.INSPECTED, self.target, f'timeout: {self.inspection.summary()}')
            self.inspection.cancel()
        self._event(Ev.DEPARTED, self.target)
        self.stops.advance()
        if self._begin_reverse(pose):
            return Command(), f'left stop {self.target}, backing up'
        self.tracker.progress.reset(
            max(self.tracker.progress.s, self.tracker.route.s[self.target])
        )
        self._set_phase(S.FOLLOWING)
        return Command(), f'left stop {self.target}'

    def _enter_approach(self, index: int, goal: bool):
        self.detour.end()
        self.target, self.goal, self.timer = index, goal, 0.0
        self.approach.reset()
        self.tracker.shift.reset()
        self._set_phase(S.APPROACHING)

    def _enter_dwell(self, note: str):
        self.timer = 0.0
        self._set_phase(S.DWELLING)
        self._event(Ev.ARRIVED, self.target, note)
        self.inspect = (self.target, self.inspection.begin())

    def _event(self, kind: Ev, index: int | None, message: str = ''):
        if index is None:
            return
        route = self.tracker.route
        dwell = float(route.dwell[index])
        self.events.append(StopEvent(kind, int(index), route.pose(index), dwell, message))
