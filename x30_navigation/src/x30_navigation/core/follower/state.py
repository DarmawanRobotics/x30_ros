"""Follower states, the transitions between them and the data the follower exchanges."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from x30_navigation.core.avoidance.base import AvoidanceOut
from x30_navigation.core.controllers.base import Command
from x30_utils.math import Pose2D


class FollowerState(Enum):
    """What the follower does; the values are the NavigationStatus constants."""

    INACTIVE = 0
    JOINING = 1
    FOLLOWING = 2
    APPROACHING = 3
    DWELLING = 4
    PAUSED = 5
    BLOCKED = 6
    OFF_PATH = 7
    GOAL_REACHED = 8
    FAILED = 9


S = FollowerState
MOVING = frozenset({S.JOINING, S.FOLLOWING, S.APPROACHING, S.DWELLING})
HALTED = frozenset({S.PAUSED, S.BLOCKED})
DONE = frozenset({S.OFF_PATH, S.GOAL_REACHED, S.FAILED})
_START = HALTED | {S.JOINING, S.FOLLOWING, S.APPROACHING}
_NEXT = {
    S.INACTIVE: _START,
    S.JOINING: HALTED | DONE | {S.FOLLOWING},
    S.FOLLOWING: HALTED | DONE | {S.APPROACHING, S.DWELLING, S.JOINING},
    S.APPROACHING: HALTED | DONE | {S.DWELLING, S.FOLLOWING, S.JOINING},
    S.DWELLING: HALTED | DONE | {S.FOLLOWING, S.JOINING},
    S.PAUSED: MOVING | HALTED | DONE,
    S.BLOCKED: MOVING | HALTED | DONE,
    S.OFF_PATH: _START,
    S.GOAL_REACHED: _START,
    S.FAILED: _START,
}
ALLOWED: dict[FollowerState, frozenset] = {
    state: frozenset(nxt | {state, S.INACTIVE, S.FAILED}) for state, nxt in _NEXT.items()
}
"""Allowed changes of the reported state; every state may stop (INACTIVE) or fail."""


class TransitionError(RuntimeError):
    """A state change the transition table does not allow (a bug)."""


def check_transition(old: FollowerState, new: FollowerState):
    """Raise TransitionError unless old -> new is allowed."""
    if new not in ALLOWED[old]:
        raise TransitionError(f'{old.name} -> {new.name} is not allowed')


class StopEventKind(Enum):
    """StopPointEvent constants."""

    ARRIVED = 0
    INSPECTED = 1
    DEPARTED = 2
    SKIPPED = 3


@dataclass(frozen=True)
class StopEvent:
    """Something that happened at a stop point."""

    kind: StopEventKind
    index: int
    pose: Pose2D
    dwell_sec: float
    message: str = ''


@dataclass(frozen=True)
class ScanInput:
    """The 360 deg safety scan: obstacle points (robot frame), blocked sectors, age (s)."""

    points: np.ndarray
    blocked: bool
    age: float
    centres: np.ndarray | None = None

    @property
    def guard_points(self) -> np.ndarray:
        """Points the motion guard checks (the sector centres when known)."""
        return self.points if self.centres is None else self.centres


@dataclass(frozen=True)
class PoseInput:
    """The robot pose from TF and how old it is (s; negative: from the future)."""

    pose: Pose2D
    age: float


@dataclass(frozen=True)
class Inputs:
    """Everything the follower reads in one control tick."""

    now: float
    dt: float
    pose: PoseInput | None = None
    scan: ScanInput | None = None
    avoidance: AvoidanceOut | None = None
    avoidance_age: float = float('inf')
    twist: Command | None = None


@dataclass(frozen=True)
class Report:
    """navigation/status content."""

    state: FollowerState
    message: str = ''
    pause_reasons: tuple[str, ...] = ()
    index: int = 0
    count: int = 0
    remaining: float = 0.0
    cross_track: float = 0.0


@dataclass(frozen=True)
class Decision:
    """The outcome of one tick: the command to publish (None: nothing) and what to report."""

    command: Command | None
    report: Report
    events: list[StopEvent] = field(default_factory=list)
    inspect: tuple[int, int] | None = None
    debug: dict = field(default_factory=dict)
