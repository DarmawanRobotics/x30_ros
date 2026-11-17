"""Docking states and the data docking_node exchanges with the state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

from x30_navigation.core.controllers.base import Command
from x30_navigation.core.docking.geometry import DockErrors
from x30_utils.math import Pose2D


class DockState(Enum):
    """What docking does; the values are the DockingStatus constants."""

    IDLE = 0
    SEARCHING = 1
    CENTERING = 2
    APPROACHING = 3
    SETTLING = 4
    BACKING_OFF = 5
    SITTING = 6
    CHARGING = 7
    DOCKED = 8
    UNDOCKING = 9
    FAILED = 10


D = DockState
ATTEMPT = frozenset({D.SEARCHING, D.CENTERING, D.APPROACHING, D.SETTLING, D.BACKING_OFF})
"""States of one docking attempt: the robot is commanded every tick."""
BUSY = ATTEMPT | {D.SITTING, D.CHARGING, D.UNDOCKING}
"""States that only cancel may interrupt."""

NO_ERRORS = DockErrors(math.nan, math.nan, math.nan)


@dataclass(frozen=True)
class PoseInput:
    """A pose in the odometry frame and how old it is (s)."""

    pose: Pose2D
    age: float


@dataclass(frozen=True)
class DockInputs:
    """Everything one tick reads."""

    now: float
    pose: PoseInput | None = None  # base_link in odom
    tag: PoseInput | None = None  # the tag in odom, yaw = normal toward the robot
    estop: bool = False
    remote: bool = False
    armed: bool = True
    charging: bool | None = None  # None: no fresh battery state


@dataclass(frozen=True)
class HookRequest:
    """Call the Trigger hooks of one group; answer with on_hooks(ticket, ...)."""

    ticket: int
    group: str  # sit, charge, stop_charge, stand


@dataclass(frozen=True)
class DockReport:
    """The state as docking/status reports it."""

    state: DockState = D.IDLE
    message: str = ''
    pause_reasons: tuple[str, ...] = ()
    tag_visible: bool = False
    tag_age: float = math.inf
    errors: DockErrors = NO_ERRORS
    attempt: int = 0
    charging: bool = False


@dataclass(frozen=True)
class DockDecision:
    """What one tick decided: a command (None: publish nothing), hooks to call, the report."""

    command: Command | None
    report: DockReport
    hooks: HookRequest | None = None
    goal: Pose2D | None = None  # the docked pose in odom, when known
    changed: bool = False  # the state changed in this tick
    notes: tuple[str, ...] = field(default_factory=tuple)  # things worth a log line


class Stopwatch:
    """Seconds spent in a phase, frozen while the robot is paused."""

    def __init__(self):
        self.elapsed = 0.0

    def reset(self):
        """Start over."""
        self.elapsed = 0.0

    def run(self, dt: float, paused: bool = False) -> float:
        """Advance by dt unless paused; return the elapsed time."""
        if not paused:
            self.elapsed += max(dt, 0.0)
        return self.elapsed


class StallWatch:
    """A robot that is commanded but does not move (held by the control field or a wall)."""

    def __init__(self, timeout_sec: float, distance: float, yaw: float):
        self.timeout, self.distance, self.yaw = timeout_sec, distance, yaw
        self.anchor: Pose2D | None = None
        self.since = 0.0

    def reset(self):
        """Forget the anchor (a new phase)."""
        self.anchor, self.since = None, 0.0

    def stalled(self, pose: Pose2D, command: Command, dt: float) -> bool:
        """Return True once the robot stood still for the timeout while commanded."""
        moving = abs(command.vx) + abs(command.vy) > 1e-3 or abs(command.wz) > 1e-3
        if not moving or self.anchor is None:
            self.anchor, self.since = pose, 0.0
            return False
        turned = abs(math.remainder(pose.yaw - self.anchor.yaw, 2.0 * math.pi))
        if self.anchor.distance(pose) > self.distance or turned > self.yaw:
            self.anchor, self.since = pose, 0.0
            return False
        self.since += max(dt, 0.0)
        return self.since >= self.timeout
