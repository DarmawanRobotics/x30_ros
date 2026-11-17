"""Configuration of getting past obstacles: path shift, detours and backing up when stuck."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import param


@dataclass(frozen=True)
class PathShiftConfig:
    """Sliding sideways around obstacles (holonomic robots)."""

    max_offset: float = param(0.8, min=0.0, max=5.0, unit='m', doc='furthest from the route')
    step: float = param(0.1, min=0.02, max=1.0, unit='m', doc='candidate offset spacing')
    look_ahead: float = param(
        2.0, min=0.1, max=20.0, unit='m', doc='route ahead of the front edge that must be free'
    )
    clearance: float = param(0.1, min=0.0, max=2.0, unit='m', doc='gap kept beside the body')
    offset_rate: float = param(
        0.3, min=0.01, max=5.0, unit='m/s', doc='how fast it slides onto a new offset'
    )
    max_lead: float = param(
        0.25, min=0.02, max=2.0, unit='m', doc='the offset leads the robot by at most this'
    )
    switch_cost: float = param(0.5, min=0.0, max=10.0, doc='keeps its side (hysteresis)')
    switch_margin: float = param(
        0.05, min=0.0, max=1.0, unit='m', doc='extra clearance a new offset needs'
    )
    return_delay_sec: float = param(
        0.5, min=0.0, max=30.0, unit='s', doc='a way back must stay proposed this long'
    )
    stop_margin: float = param(
        0.4, min=0.0, max=5.0, unit='m', doc='stop this far before an obstacle (front edge)'
    )
    patience_sec: float = param(
        3.0, min=0.0, max=600.0, unit='s', doc='held this long with no clear offset: search'
    )
    search_offset: float = param(
        1.5, min=0.0, max=5.0, unit='m', doc='furthest while searching (<= max_offset: off)'
    )
    emergency_slide: bool = param(
        True, doc='an emergency holds the forward motion only: the robot still slides out'
    )
    centre_room: float = param(
        0.25,
        min=0.0,
        max=2.0,
        unit='m',
        doc='a lane with less room beside the body is driven in its middle (<= clearance: off)',
    )


@dataclass(frozen=True)
class DetourConfig:
    """Turning in place toward a gap and driving through it when sliding does not get on."""

    enabled: bool = param(True, doc='turn toward a gap and drive through it when stuck')
    after_sec: float = param(
        1.5, min=0.5, max=600.0, unit='s', doc='stuck this long while following: detour'
    )
    max_angle_deg: float = param(
        75.0, min=5.0, max=135.0, unit='deg', doc='headings this far off the route direction'
    )
    length: float = param(2.0, min=0.3, max=10.0, unit='m', doc='longest straight run')
    min_progress: float = param(
        0.3, min=0.0, max=5.0, unit='m', doc='a run must take the robot this far along the route'
    )
    speed: float = param(0.25, min=0.05, max=1.0, unit='m/s', doc='speed on the run')


@dataclass(frozen=True)
class BacktrackConfig:
    """Backing up along the driven path when the robot gets no further along the route."""

    enabled: bool = param(True, doc='back up along the driven path when stuck, then try again')
    after_sec: float = param(
        30.0, min=5.0, max=600.0, unit='s', doc='no progress along the route this long: back up'
    )
    min_progress: float = param(
        0.3, min=0.05, max=5.0, unit='m', doc='less progress than this counts as no progress'
    )
    distance: float = param(
        1.0, min=0.2, max=5.0, unit='m', doc='how far back along the driven path'
    )
    speed: float = param(
        0.15, min=0.05, max=0.5, unit='m/s', doc='backing-up speed (360 deg guard, rear sectors)'
    )
    max_attempts: int = param(
        3, min=1, max=20, doc='attempts at one place, then it stays BLOCKED (waits)'
    )
