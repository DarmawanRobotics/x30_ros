"""One goal of the planner, from planning to arrival: what to do when the follower reports."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from x30_navigation.core.config.planner import MonitorConfig
from x30_navigation.core.follower.state import FollowerState
from x30_navigation.core.planning.planner import Plan, remaining_length
from x30_utils.math import Pose2D


class PlannerState(Enum):
    """Planner states; the values are the PlannerStatus constants."""

    IDLE = 0
    PLANNING = 1
    FOLLOWING = 2
    REPLANNING = 3
    ARRIVED = 4
    FAILED = 5
    CANCELLED = 6


P = PlannerState
ACTIVE = frozenset({P.PLANNING, P.FOLLOWING, P.REPLANNING})


class Action(Enum):
    """What the node does after a monitor step."""

    NONE = 0
    STOP = 1
    REPLAN = 2


@dataclass(frozen=True)
class Step:
    """The action and the reason for it."""

    action: Action
    reason: str = ''


class GoalSession:
    """State of the current goal."""

    def __init__(self, config: MonitorConfig):
        self.config = config
        self.state = P.IDLE
        self.message = ''
        self.goal: Pose2D | None = None
        self.plan: Plan | None = None
        self.path = np.zeros((0, 2))
        self.replans = 0
        self.sent_at = -math.inf
        self.blocked_since: float | None = None
        self.inactive_since: float | None = None
        self.cancel_requested = False

    @property
    def active(self) -> bool:
        """True from planning until the goal ends."""
        return self.state in ACTIVE

    def begin(self, goal: Pose2D):
        """Start a new goal."""
        self.goal, self.plan, self.path = goal, None, np.zeros((0, 2))
        self.replans, self.cancel_requested = 0, False
        self._set(P.PLANNING, f'to {self.goal_text()}')

    def replanning(self, reason: str):
        """Count a replan."""
        self.replans += 1
        self._set(P.REPLANNING, f'to {self.goal_text()}: {reason}')

    def following(self, plan: Plan, now: float, note: str = ''):
        """Record that the follower took the plan and started."""
        self.plan, self.sent_at = plan, now
        self.path = np.array([(w.x, w.y) for w in plan.waypoints])
        self.blocked_since = self.inactive_since = None
        self._set(P.FOLLOWING, f'to {self.goal_text()}: {note or plan.message}')

    def end(self, state: PlannerState, reason: str):
        """End the goal (ARRIVED, FAILED or CANCELLED)."""
        self._set(state, f'to {self.goal_text()}: {reason}')

    def goal_text(self) -> str:
        """Describe the goal."""
        g = self.goal
        return 'nothing' if g is None else f'({g.x:.2f}, {g.y:.2f})'

    def remaining(self, pose: Pose2D | None) -> float:
        """Return the path length left (m)."""
        if self.plan is None:
            return 0.0
        if pose is None or not self.active:
            return 0.0 if self.state is P.ARRIVED else self.plan.length
        return remaining_length(self.path, pose.x, pose.y)

    def monitor(self, nav: FollowerState | None, received: float, now: float, pose) -> Step:
        """Return what to do for the follower state received at time received."""
        if self.state is not P.FOLLOWING or nav is None or received < self.sent_at:
            return Step(Action.NONE)
        cfg = self.config
        if nav is FollowerState.GOAL_REACHED and self._at_goal(pose):
            self.end(P.ARRIVED, 'arrived')
            return Step(Action.STOP if cfg.stop_on_arrival else Action.NONE)
        if nav is FollowerState.FAILED:
            self.end(P.FAILED, 'the follower failed')
            return Step(Action.STOP)
        if nav is FollowerState.OFF_PATH:
            return self._replan('off the path')
        self.blocked_since = self._since(nav is FollowerState.BLOCKED, self.blocked_since, now)
        if self.blocked_since is not None and now - self.blocked_since >= cfg.replan_after_sec:
            self.blocked_since = None
            return self._replan('blocked')
        inactive = nav is FollowerState.INACTIVE
        self.inactive_since = self._since(inactive, self.inactive_since, now)
        if self.inactive_since is not None and now - self.inactive_since > cfg.cancel_after_sec:
            self.end(P.CANCELLED, 'navigation was stopped')
        return Step(Action.NONE)

    def _replan(self, reason: str) -> Step:
        if self.replans >= self.config.max_replans:
            self.end(P.FAILED, f'{reason}, gave up after {self.replans} replans')
            return Step(Action.STOP, reason)
        return Step(Action.REPLAN, reason)

    def _at_goal(self, pose: Pose2D | None) -> bool:
        goal = self.plan.goal if self.plan is not None and self.plan.goal else self.goal
        return pose is not None and pose.distance(goal) <= self.config.arrival_tolerance

    @staticmethod
    def _since(condition: bool, since: float | None, now: float) -> float | None:
        if not condition:
            return None
        return now if since is None else since

    def _set(self, state: PlannerState, message: str):
        self.state, self.message = state, message
