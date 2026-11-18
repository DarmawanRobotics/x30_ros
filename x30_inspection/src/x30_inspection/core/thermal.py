"""Overheat analysis of one target: absolute, ambient and similar criteria (NETA MTS practice)."""

# - absolute: the hottest temperature itself (a target's max_temp_c is its critical limit)
# - ambient: the hottest temperature over the air temperature at the site
# - similar: the hottest region over the median of the other regions (the three phases of a
#   disconnector, three clamps of a busbar joint): only with two regions or more
# The severity of the target is the highest of its findings.

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import statistics

from x30_inspection.core.config import ThermalConfig
from x30_inspection.core.plan import Region, Target


class Severity(IntEnum):
    """Finding severities (x30_msgs/InspectionFinding)."""

    NORMAL = 0
    ATTENTION = 1
    WARNING = 2
    CRITICAL = 3


@dataclass(frozen=True)
class Reading:
    """The camera's temperatures of one region (degC; where the hottest pixel is)."""

    region: str
    temp_max: float
    temp_min: float
    max_x: int = 0
    max_y: int = 0


@dataclass(frozen=True)
class Finding:
    """One result of a target (x30_msgs/InspectionFinding without the files)."""

    target: str
    sensor: str
    criterion: str
    label: str
    severity: Severity
    value: float
    reference: float = 0.0
    delta: float = 0.0
    unit: str = 'degC'
    message: str = ''


FRAME = 'frame'


def regions_of(target: Target) -> tuple[Region, ...]:
    """Return the boxes to measure: explicit regions, N strips, or none (the whole frame)."""
    if target.thermal.regions:
        return target.thermal.regions
    n = target.thermal.strips
    if n < 2:
        return ()
    return tuple(Region(f'strip{i + 1}', (i / n, 0.0, (i + 1) / n, 1.0)) for i in range(n))


def level(value: float, attention: float, warning: float, critical: float) -> Severity:
    """Return the severity of a value against its thresholds."""
    if value >= critical:
        return Severity.CRITICAL
    if value >= warning:
        return Severity.WARNING
    if value >= attention:
        return Severity.ATTENTION
    return Severity.NORMAL


def absolute_limits(config: ThermalConfig, max_temp_c: float) -> tuple[float, float, float]:
    """Return the absolute thresholds, with a target's own critical limit when it has one."""
    a = config.absolute
    if max_temp_c <= 0.0:
        return a.attention, a.warning, a.critical
    return min(a.attention, max_temp_c), min(a.warning, max_temp_c), max_temp_c


def median_reading(samples: list[Reading]) -> Reading:
    """Return the median of repeated readings of one region (the max of the median sample)."""
    ordered = sorted(samples, key=lambda r: r.temp_max)
    middle = ordered[len(ordered) // 2]
    low = statistics.median(r.temp_min for r in samples)
    return Reading(middle.region, middle.temp_max, low, middle.max_x, middle.max_y)


def analyse(target: Target, readings: list[Reading], config: ThermalConfig) -> list[Finding]:
    """Return the findings of a target from the readings of its regions."""
    if not readings:
        return []
    hottest = max(readings, key=lambda r: r.temp_max)
    t = hottest.temp_max
    where = f'{hottest.region} at ({hottest.max_x}, {hottest.max_y})'
    found = []
    limits = absolute_limits(config, target.thermal.max_temp_c)
    sev = level(t, *limits)
    found.append(Finding(target.name, 'thermal', 'absolute', hottest.region, sev, t,
                         limits[2], t - limits[2], message=f'{t:.1f} degC, {where}'))  # fmt: skip
    ambient = config.ambient_c
    rise = t - ambient
    a = config.ambient
    sev = level(rise, a.attention, a.warning, a.critical)
    text = f'{rise:+.1f} degC over the ambient {ambient:.1f}'
    found.append(Finding(target.name, 'thermal', 'ambient', hottest.region, sev, t, ambient,
                         rise, message=text))  # fmt: skip
    others = [r.temp_max for r in readings if r is not hottest]
    if others:
        reference = statistics.median(others)
        delta = t - reference
        s = config.similar
        sev = level(delta, s.attention, s.warning, s.critical)
        text = f'{hottest.region} {delta:+.1f} degC over the others'
        found.append(Finding(target.name, 'thermal', 'similar', hottest.region, sev, t,
                             reference, delta, message=text))  # fmt: skip
    return found


def worst(findings: list[Finding]) -> Severity:
    """Return the highest severity of findings (NORMAL for none)."""
    return max((f.severity for f in findings), default=Severity.NORMAL)
