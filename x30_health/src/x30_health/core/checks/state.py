"""Typed states of the stack: odometry, localization (#9: NONE warns) and control."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from x30_health.core.checks.base import Check, Issue, Level, fmt
from x30_health.core.config import StateConfig


class OdometryState(IntEnum):
    """OdometryStatus.state."""

    INITIALIZING = 0
    OK = 1
    DEGRADED = 2
    LOST = 3


class LocalizationState(IntEnum):
    """LocalizationStatus.state."""

    NONE = 0
    LOCALIZED = 1
    UNCERTAIN = 2


class Field(IntEnum):
    """ControlStatus.field."""

    OFF = 0
    CLEAR = 1
    LIMITED = 2
    STOP = 3
    STALE = 4
    BLIND = 5


@dataclass(frozen=True)
class OdometryInput:
    """The fields of an OdometryStatus that health reads."""

    state: int
    message: str = ''


@dataclass(frozen=True)
class LocalizationInput:
    """The fields of a LocalizationStatus that health reads."""

    state: int
    sigma_xy: float = 0.0
    sigma_yaw: float = 0.0
    message: str = ''


@dataclass(frozen=True)
class ControlInput:
    """The fields of a ControlStatus that health reads."""

    estop: bool
    estop_reasons: tuple[str, ...] = ()
    field: int = Field.OFF


def _enum(cls, value: int):
    try:
        return cls(value)
    except ValueError:
        return None


def odometry_issue(inp: OdometryInput) -> Issue:
    """Return the finding of an odometry status (LOST is an error)."""
    state = _enum(OdometryState, inp.state)
    detail = f': {inp.message}' if inp.message else ''
    if state is None:
        return Issue(Level.WARN, 'state:odometry', f'odometry state {inp.state} unknown')
    level = {
        OdometryState.OK: Level.OK,
        OdometryState.LOST: Level.ERROR,
    }.get(state, Level.WARN)
    return Issue(level, 'state:odometry', f'odometry {state.name.lower()}{detail}')


def localization_issue(inp: LocalizationInput) -> Issue:
    """Return the finding of a localization status (NONE and UNCERTAIN warn)."""
    state = _enum(LocalizationState, inp.state)
    if state is None:
        return Issue(Level.WARN, 'state:localization', f'localization state {inp.state} unknown')
    values = {'sigma_xy': inp.sigma_xy, 'sigma_yaw': inp.sigma_yaw}
    if state is LocalizationState.LOCALIZED:
        text = f'localized, sigma {fmt(inp.sigma_xy, 2)} m'
        return Issue(Level.OK, 'state:localization', text, values)
    text = 'not localized' if state is LocalizationState.NONE else 'localization uncertain'
    if inp.message:
        text = f'{text}: {inp.message}'
    return Issue(Level.WARN, 'state:localization', text, values)


def control_issue(inp: ControlInput) -> Issue:
    """Return the finding of a control status (e-stop, stale or blind field are errors)."""
    if inp.estop:
        reasons = ', '.join(inp.estop_reasons) or 'no reason given'
        return Issue(Level.ERROR, 'state:control', f'e-stop engaged ({reasons})')
    field = _enum(Field, inp.field)
    if field is Field.STALE:
        return Issue(Level.ERROR, 'state:control', 'protective field: no fresh safety scan')
    if field is Field.BLIND:
        return Issue(Level.ERROR, 'state:control', 'protective field: safety scan blind')
    name = field.name.lower() if field is not None else str(inp.field)
    return Issue(Level.OK, 'state:control', f'control ok, field {name}')


class StateCheck(Check):
    """One finding per enabled status topic; missing or stale statuses warn."""

    def __init__(self, config: StateConfig):
        self.config = config
        self.latest: dict[str, tuple[float, Issue]] = {}

    def configure(self, config: StateConfig):
        """Apply live changes."""
        self.config = config

    def on_odometry(self, now: float, inp: OdometryInput):
        """Store an odometry status received at monotonic now."""
        self.latest['odometry'] = (now, odometry_issue(inp))

    def on_localization(self, now: float, inp: LocalizationInput):
        """Store a localization status received at monotonic now."""
        self.latest['localization'] = (now, localization_issue(inp))

    def on_control(self, now: float, inp: ControlInput):
        """Store a control status received at monotonic now."""
        self.latest['control'] = (now, control_issue(inp))

    def run(self, now: float) -> list[Issue]:
        """Return one finding per enabled status topic."""
        c = self.config
        topics = (
            ('odometry', c.odometry_topic),
            ('localization', c.localization_topic),
            ('control', c.control_topic),
        )
        return [self._check(key, topic, now) for key, topic in topics if topic]

    def _check(self, key: str, topic: str, now: float) -> Issue:
        name = f'state:{key}'
        if key not in self.latest:
            return Issue(Level.WARN, name, f'no {topic} received')
        at, issue = self.latest[key]
        age = now - at
        if age > self.config.timeout_sec:
            return Issue(Level.WARN, name, f'{topic} stale ({fmt(age)} s, publisher down?)')
        return issue
