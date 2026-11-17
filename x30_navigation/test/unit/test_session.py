"""Planner goal session: arrival on the snapped goal, replans, failures, cancel."""

from __future__ import annotations

import pytest

from x30_navigation.core.config.planner import MonitorConfig
from x30_navigation.core.follower.state import FollowerState as F
from x30_navigation.core.planning.planner import Plan
from x30_navigation.core.planning.session import Action, GoalSession, PlannerState
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D

CONFIG = MonitorConfig(replan_after_sec=1.0, max_replans=2, cancel_after_sec=0.5)


GOAL = Pose2D(5.0, 0.0)


def following(goal=GOAL, snapped=None):
    session = GoalSession(CONFIG)
    session.begin(goal)
    end = snapped or goal
    plan = Plan(True, 'ok', [RoutePoint(0.0, 0.0), RoutePoint(end.x, end.y)], end.x, end)
    session.following(plan, now=10.0)
    return session


def test_arrival_is_tested_against_the_snapped_goal():
    """F8: a goal moved out of an obstacle arrives where the plan ends."""
    session = following(Pose2D(5.0, 0.0), snapped=Pose2D(4.4, 0.0))
    step = session.monitor(F.GOAL_REACHED, 11.0, 11.0, Pose2D(4.4, 0.05))
    assert session.state is PlannerState.ARRIVED and step.action is Action.STOP


def test_stale_status_and_far_goal_reached_are_ignored():
    session = following()
    assert session.monitor(F.GOAL_REACHED, 9.0, 11.0, Pose2D(5.0, 0.0)).action is Action.NONE
    session.monitor(F.GOAL_REACHED, 11.0, 11.0, Pose2D(2.0, 0.0))
    assert session.state is PlannerState.FOLLOWING


def test_blocked_replans_then_gives_up_and_stops():
    session = following()
    assert session.monitor(F.BLOCKED, 11.0, 11.0, None).action is Action.NONE
    assert session.monitor(F.BLOCKED, 12.1, 12.1, None).action is Action.REPLAN
    session.replanning('blocked')
    session.replanning('blocked')
    session.following(session.plan, now=13.0)
    step = session.monitor(F.OFF_PATH, 13.5, 13.5, None)
    assert step.action is Action.STOP and session.state is PlannerState.FAILED


def test_follower_failure_stops_and_fails():
    """F9: a failure after set_path stops the follower."""
    session = following()
    assert session.monitor(F.FAILED, 11.0, 11.0, None).action is Action.STOP
    assert session.state is PlannerState.FAILED


def test_stopped_by_someone_else_is_cancelled():
    session = following()
    session.monitor(F.INACTIVE, 11.0, 11.0, None)
    session.monitor(F.INACTIVE, 11.6, 11.6, None)
    assert session.state is PlannerState.CANCELLED


def test_remaining_length():
    session = following()
    assert session.remaining(Pose2D(1.0, 0.0)) == pytest.approx(4.0)
    session.end(PlannerState.ARRIVED, 'x')
    assert session.remaining(None) == 0.0
