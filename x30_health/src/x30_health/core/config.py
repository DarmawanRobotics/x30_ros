"""Configuration of health_monitor_node (HealthConfig and its sections)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_health.core.specs import (
    collect_problems,
    parse_edge,
    parse_rule,
    parse_topic,
    parse_topics,
)
from x30_utils.config import nested, param


@dataclass(frozen=True)
class SensorsConfig:
    """Watched topics: rate and header stamp age, subscribed raw (no deserialisation)."""

    topics: tuple[str, ...] = param(
        ('/livox/lidar:5.0', '/Odometry:8.0'),
        live=False,
        doc='"topic:min_hz[:max_age_sec]" (max_age_sec 0: no stamp check)',
    )
    rate_tolerance: float = param(
        0.8, min=0.0, max=1.0, doc='a rate below this share of min_hz is an issue'
    )
    window_sec: float = param(5.0, min=0.5, max=60.0, unit='s', live=False, doc='rate window')
    max_age_sec: float = param(
        0.5, min=0.0, max=60.0, unit='s', doc='stamp age limit of entries without one; 0: off'
    )
    max_future_sec: float = param(
        0.1, min=0.0, max=60.0, unit='s', doc='stamps further ahead of the clock: clock offset'
    )
    discover_period_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', live=False, doc='look for unsubscribed topics'
    )

    def validate(self) -> list[str]:
        """Return problems of the topic entries."""
        problems = collect_problems(parse_topic, self.topics)
        if not problems:
            try:
                parse_topics(self.topics)
            except ValueError as error:
                problems.append(str(error))
        return [f'topics: {p}' for p in problems]


@dataclass(frozen=True)
class SystemConfig:
    """The computer: temperature, memory, disk, load, CPU and Jetson GPU; a limit of 0 is off."""

    thermal_zones: tuple[str, ...] = param(
        ('cpu*', 'gpu*', 'tj*', 'x86_pkg_temp'),
        doc='thermal zone types read (fnmatch, any case); others are ignored, some are bogus',
    )
    max_temp_c: float = param(85.0, min=0.0, max=150.0, unit='degC', doc='hottest zone')
    max_memory_percent: float = param(92.0, min=0.0, max=100.0, unit='%', doc='memory in use')
    min_disk_free_gb: float = param(5.0, min=0.0, max=1e6, unit='GB', doc='free at disk_path')
    disk_path: str = param('~', doc='file system to watch (~ and $VAR expanded)')
    max_load_per_cpu: float = param(2.0, min=0.0, max=100.0, doc='1 min load average / CPUs')
    max_cpu_percent: float = param(0.0, min=0.0, max=100.0, unit='%', doc='CPU busy share')
    max_gpu_percent: float = param(0.0, min=0.0, max=100.0, unit='%', doc='Jetson GPU load')


@dataclass(frozen=True)
class NodesConfig:
    """Nodes that must run, by fully qualified name."""

    expected: tuple[str, ...] = param((), doc='"/namespace/name" of every node that must run')

    def validate(self) -> list[str]:
        """Return names that are not fully qualified."""
        return [
            f'expected: {n!r} is not fully qualified ("/namespace/name")'
            for n in self.expected
            if not n.startswith('/') or n.endswith('/') or '//' in n
        ]


@dataclass(frozen=True)
class StateConfig:
    """Typed status topics of the stack and the safety scan; an empty topic is off."""

    odometry_topic: str = param('odometry/status', live=False, doc='OdometryStatus')
    localization_topic: str = param('localization/status', live=False, doc='LocalizationStatus')
    control_topic: str = param('control/status', live=False, doc='ControlStatus')
    safety_scan_topic: str = param('perception/safety_scan', live=False, doc='SectorScan (360)')
    timeout_sec: float = param(
        3.0, min=0.1, max=600.0, unit='s', doc='a status older than this is stale'
    )
    scan_timeout_sec: float = param(
        1.0, min=0.05, max=60.0, unit='s', doc='a safety scan older than this is stale'
    )


@dataclass(frozen=True)
class TfConfig:
    """Age of every dynamic TF edge, with a clock offset told apart from late delivery."""

    enabled: bool = param(True, live=False, doc='listen to TF')
    edges: tuple[str, ...] = param(
        (), doc='"parent->child" dynamic edges that must exist (every dynamic edge is watched)'
    )
    max_age_sec: float = param(0.5, min=0.01, max=60.0, unit='s', doc='newest transform age')
    max_future_sec: float = param(
        0.1, min=0.0, max=60.0, unit='s', doc='stamps further ahead of the clock: clock offset'
    )
    window_sec: float = param(5.0, min=1.0, max=60.0, unit='s', doc='age history per edge')
    offset_jitter_sec: float = param(
        0.2, min=0.0, max=10.0, unit='s', doc='late ages steadier than this: a clock offset'
    )

    def validate(self) -> list[str]:
        """Return problems of the edge entries."""
        return [f'edges: {p}' for p in collect_problems(parse_edge, self.edges)]


@dataclass(frozen=True)
class ClockConfig:
    """Optional NTP offset from `chronyc tracking`."""

    enabled: bool = param(False, live=False, doc='run the command')
    command: str = param('chronyc tracking', live=False, doc='prints "System time : x seconds"')
    period_sec: float = param(10.0, min=1.0, max=3600.0, unit='s', doc='run it this often')
    timeout_sec: float = param(1.0, min=0.1, max=10.0, unit='s', doc='command timeout')
    max_offset_sec: float = param(0.05, min=0.0, max=60.0, unit='s', doc='|offset| limit')


@dataclass(frozen=True)
class DegradationConfig:
    """Speed scale on health/status: errors 0, warnings by rule, otherwise 1."""

    rules: tuple[str, ...] = param(
        ('sensor:/livox/lidar=0.5', 'sensor:/Odometry=0.5', 'tf:*=0.5'),
        doc='"<issue name pattern>=<scale>" for warnings (fnmatch); the lowest match wins',
    )
    warn_scale: float = param(1.0, min=0.0, max=1.0, doc='scale of warnings without a rule')

    def validate(self) -> list[str]:
        """Return problems of the rule entries."""
        return [f'rules: {p}' for p in collect_problems(parse_rule, self.rules)]


@dataclass(frozen=True)
class HealthConfig:
    """health_monitor_node: what is checked and what health/status asks for."""

    period_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', live=False, doc='health/status and diagnostics period'
    )
    startup_grace_sec: float = param(
        30.0, min=0.0, max=600.0, unit='s', doc='missing nodes and inputs only warn this long'
    )
    diagnostics_prefix: str = param('health', doc='"<prefix>: <check>" in rqt_robot_monitor')
    extra_checks: tuple[str, ...] = param(
        (), live=False, doc='registered checks to add (entry points x30_health.checks)'
    )
    sensors: SensorsConfig = nested(SensorsConfig)
    system: SystemConfig = nested(SystemConfig)
    nodes: NodesConfig = nested(NodesConfig)
    state: StateConfig = nested(StateConfig)
    tf: TfConfig = nested(TfConfig)
    clock: ClockConfig = nested(ClockConfig)
    degradation: DegradationConfig = nested(DegradationConfig)
