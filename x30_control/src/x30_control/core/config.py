"""Configuration of the control mux: arbitration and protective field (read-only at runtime)."""

from __future__ import annotations

from dataclasses import dataclass
import re

from x30_utils.config import param

MODES = ('auto', 'remote')
SOURCE_RE = re.compile(r'^[a-zA-Z][a-zA-Z0-9_]*$')


@dataclass(frozen=True)
class MuxConfig:
    """Who may drive, how long a source may stay silent, e-stop hold and re-arm."""

    mode: str = param('auto', choices=MODES, live=False, doc='mode at start')
    remote_sources: tuple[str, ...] = param(
        ('handheld', 'dashboard'),
        live=False,
        doc='operator sources, each on control/remote/<source>/cmd_vel; the first is the default',
    )
    nav_timeout_sec: float = param(
        0.5, min=0.02, max=5.0, unit='s', live=False, doc='zero once when navigation is silent'
    )
    remote_timeout_sec: float = param(
        0.3, min=0.02, max=5.0, unit='s', live=False, doc='zero once when the operator is silent'
    )
    estop_hold_hz: float = param(
        10.0, min=1.0, max=100.0, unit='Hz', live=False, doc='zero commands while engaged'
    )
    rearm_neutral_count: int = param(
        3,
        min=1,
        max=100,
        live=False,
        doc='consecutive neutral commands the source must send before it drives again',
    )
    neutral_eps: float = param(
        0.01, min=1e-4, max=0.2, unit='m/s, rad/s', live=False, doc='neutral: every axis below'
    )
    hw_estop_timeout_sec: float = param(
        0.0,
        min=0.0,
        max=10.0,
        unit='s',
        live=False,
        doc='engage when control/hw_estop is silent this long (0: the input is optional)',
    )
    use_health_limit: bool = param(
        False, live=False, doc='scale every command by health/status speed_scale'
    )
    health_timeout_sec: float = param(
        3.0, min=0.1, max=60.0, unit='s', live=False, doc='older health/status is ignored'
    )
    status_hz: float = param(
        2.0, min=0.2, max=50.0, unit='Hz', live=False, doc='control/status period (also on change)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = [
            f'remote_sources: {name!r} is not a valid topic name token'
            for name in self.remote_sources
            if not SOURCE_RE.match(name)
        ]
        if len(set(self.remote_sources)) != len(self.remote_sources):
            problems.append('remote_sources: duplicate names')
        if self.mode == 'remote' and not self.remote_sources:
            problems.append('mode remote needs at least one remote source')
        return problems


@dataclass(frozen=True)
class FieldConfig:
    """Protective field: margins, minimum speeds, scan freshness and degraded REMOTE driving."""

    in_remote: bool = param(True, live=False, doc='apply the field in REMOTE too (remote_margin)')
    margin: float = param(
        0.15, min=0.0, max=2.0, unit='m', live=False, doc='gap kept to an obstacle in AUTO'
    )
    remote_margin: float = param(
        0.10, min=0.0, max=2.0, unit='m', live=False, doc='gap kept to an obstacle in REMOTE'
    )
    lateral_margin: float = param(
        0.10, min=0.0, max=1.0, unit='m', live=False, doc='extra width of the swept path'
    )
    human_speed: float = param(
        0.5,
        min=0.0,
        max=3.0,
        unit='m/s',
        live=False,
        doc='assumed speed of a person walking toward the robot (ISO/TS 15066 separation)',
    )
    yaw_margin: float = param(
        0.10, min=0.0, max=1.0, unit='rad', live=False, doc='turn kept short of a contact'
    )
    min_speed: float = param(
        0.02, min=0.0, max=0.5, unit='m/s', live=False, doc='a slower allowed speed stops instead'
    )
    min_yaw_rate: float = param(
        0.05, min=0.0, max=1.0, unit='rad/s', live=False, doc='a slower allowed turn stops instead'
    )
    remote_degraded_speed: float = param(
        0.15,
        min=0.0,
        max=1.0,
        unit='m/s',
        live=False,
        doc='REMOTE with a stale or blind scan: fastest point of the body',
    )
    scan_timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='scan receive age (monotonic) limit'
    )
    scan_max_age_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='scan header stamp age limit'
    )
    scan_clock_tolerance_sec: float = param(
        0.1, min=0.0, max=1.0, unit='s', live=False, doc='accepted negative stamp age (clocks)'
    )
