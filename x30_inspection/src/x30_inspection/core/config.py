"""Configuration of inspection_node: sites and reports, the cameras, thermal criteria, timing."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

SEVERITIES = ('normal', 'attention', 'warning', 'critical')


def ordered(levels) -> list[str]:
    """Return a problem unless attention <= warning <= critical."""
    ok = levels.attention <= levels.warning <= levels.critical
    return [] if ok else ['attention <= warning <= critical']


@dataclass(frozen=True)
class AbsoluteLevels:
    """Hottest temperature itself (degC; a target's max_temp_c replaces critical)."""

    attention: float = param(80.0, min=0.0, max=500.0, unit='degC', doc='possible deficiency')
    warning: float = param(90.0, min=0.0, max=500.0, unit='degC', doc='probable deficiency')
    critical: float = param(105.0, min=0.0, max=500.0, unit='degC', doc='major deficiency')

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return ordered(self)


@dataclass(frozen=True)
class AmbientLevels:
    """Hottest temperature over the ambient (NETA MTS: 1-10, 11-20, 21-40, > 40 degC)."""

    attention: float = param(11.0, min=0.0, max=500.0, unit='degC', doc='possible deficiency')
    warning: float = param(21.0, min=0.0, max=500.0, unit='degC', doc='probable deficiency')
    critical: float = param(41.0, min=0.0, max=500.0, unit='degC', doc='major deficiency')

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return ordered(self)


@dataclass(frozen=True)
class SimilarLevels:
    """Hottest of similar components over the others (NETA MTS: 1-3, 4-15, > 15 degC)."""

    attention: float = param(2.0, min=0.0, max=500.0, unit='degC', doc='possible deficiency')
    warning: float = param(4.0, min=0.0, max=500.0, unit='degC', doc='probable deficiency')
    critical: float = param(16.0, min=0.0, max=500.0, unit='degC', doc='major deficiency')

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return ordered(self)


@dataclass(frozen=True)
class ThermalConfig:
    """How temperatures are measured and judged."""

    ambient_c: float = param(
        30.0, min=-40.0, max=60.0, unit='degC', doc='air temperature at the site (live: set it)'
    )
    samples: int = param(3, min=1, max=20, doc='measurements per region; the median is used')
    sample_interval_sec: float = param(
        0.3, min=0.0, max=5.0, unit='s', doc='between two measurements'
    )
    max_age_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', doc='a thermometry frame at most this old per sample'
    )
    absolute: AbsoluteLevels = nested(AbsoluteLevels)
    ambient: AmbientLevels = nested(AmbientLevels)
    similar: SimilarLevels = nested(SimilarLevels)


@dataclass(frozen=True)
class CameraConfig:
    """The HIKMICRO PTZ camera (hikmicro_driver) and how long it takes to settle."""

    namespace: str = param('/hikmicro', live=False, doc='hikmicro_driver namespace')
    move_timeout_sec: float = param(
        20.0, min=1.0, max=120.0, unit='s', doc='a move to a target: arrived or failed by then'
    )
    call_timeout_sec: float = param(
        5.0, min=0.1, max=60.0, unit='s', doc='each temperature or snapshot call'
    )
    settle_sec: float = param(
        1.0, min=0.0, max=30.0, unit='s', doc='after arriving, before measuring (focus, image)'
    )
    park_preset: int = param(
        0, min=0, max=300, doc='camera preset called after a point; 0: the camera stays'
    )
    state_max_age_sec: float = param(
        3.0, min=0.1, max=60.0, unit='s', doc='ptz/state older than this: not connected'
    )


@dataclass(frozen=True)
class AcousticConfig:
    """The Hertzino HZ-FA-371 (hertzino_driver) and how its results are judged."""

    enabled: bool = param(True, doc='false: acoustic targets are skipped')
    namespace: str = param('/hertzino', live=False, doc='hertzino_driver namespace')
    duration_sec: float = param(
        5.0, min=1.0, max=60.0, unit='s', doc='detection time when the target sets none'
    )
    margin_sec: float = param(
        20.0, min=1.0, max=120.0, unit='s', doc='wait this long past the detection time'
    )
    status_max_age_sec: float = param(
        5.0, min=0.5, max=60.0, unit='s', doc='status older than this: not connected'
    )
    corona: str = param('attention', choices=SEVERITIES, doc='severity of corona discharge')
    surface: str = param('warning', choices=SEVERITIES, doc='severity of surface discharge')
    floating: str = param('critical', choices=SEVERITIES, doc='severity of floating discharge')
    intensity_warning: float = param(
        0.0, min=0.0, max=1000.0, doc='a discharge at least this intense is a warning; 0: off'
    )
    intensity_critical: float = param(
        0.0, min=0.0, max=1000.0, doc='a discharge at least this intense is critical; 0: off'
    )


@dataclass(frozen=True)
class InspectionConfig:
    """Everything inspection_node reads."""

    enabled: bool = param(True, doc='false: stop points pass without inspection')
    sites_dir: str = param(
        '~/x30/map', live=False, doc='<site>/inspection.yaml holds the points and targets'
    )
    reports_dir: str = param('~/x30/reports', live=False, doc='one folder per inspected point')
    reports_keep_days: int = param(
        0, min=0, max=36500, unit='d', doc='days of reports kept; 0: until the disk needs room'
    )
    reports_min_free_gb: float = param(
        6.0,
        min=0.0,
        max=1e6,
        unit='GB',
        doc='the oldest days of reports go while less is free (x30_health warns at 5); 0: never',
    )
    match_radius_m: float = param(
        1.5, min=0.1, max=10.0, unit='m', doc='a stop this close to a point inspects it'
    )
    max_duration_sec: float = param(
        150.0, min=5.0, max=1800.0, unit='s', doc='one point; later targets are skipped'
    )
    stop_max_age_sec: float = param(
        60.0, min=1.0, max=3600.0, unit='s', doc='a stop point event older than this is stale'
    )
    camera: CameraConfig = nested(CameraConfig)
    thermal: ThermalConfig = nested(ThermalConfig)
    acoustic: AcousticConfig = nested(AcousticConfig)
