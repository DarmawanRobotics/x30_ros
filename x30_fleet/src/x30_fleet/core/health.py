"""The robot's health as NETRA keeps it: level, issues, topic rates, the computer's figures."""

# x30_health publishes one diagnostic status per finding, '<prefix>: <finding>' (system:cpu,
# sensor:/livox/lidar, ...) with its numbers as key / value text; NETRA reads the names below.

from __future__ import annotations

import math

LEVELS = {0: 'ok', 1: 'warn', 2: 'error'}  # x30_msgs/HealthStatus
SENSOR_PREFIX = 'sensor:'
# finding -> (NETRA field, key of the finding's values, digits)
FIGURES = (
    ('system:temperature', 'cpu_temp_c', 'value', 1),
    ('system:memory', 'memory_percent', 'value', 1),
    ('system:memory', 'memory_used_gb', 'used_gb', 1),
    ('system:memory', 'memory_total_gb', 'total_gb', 1),
    ('system:cpu', 'cpu_percent', 'value', 1),
    ('system:gpu', 'gpu_percent', 'value', 1),
    ('system:disk', 'disk_free_gb', 'value', 1),
)


def text_number(value) -> float | None:
    """Return a number written as text, None when it is none."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def findings(statuses) -> dict:
    """Return {finding: {key: text}} of diagnostic statuses (name, values of key / value)."""
    out = {}
    for name, values in statuses:
        _, sep, rest = name.partition(': ')
        out[rest if sep else name] = values
    return out


def summary(level: int, issues, speed_scale: float, found: dict) -> dict:
    """Return the telemetry `health`: ok, level, issues, speed_scale, topics, the computer."""
    out = {
        'ok': level == 0,
        'level': LEVELS.get(level, 'error'),
        'issues': [str(i) for i in issues],
        'speed_scale': round(float(speed_scale), 2) if math.isfinite(speed_scale) else 0.0,
        'topics': {},
    }
    for name, values in found.items():
        if name.startswith(SENSOR_PREFIX):
            hz = text_number(values.get('hz'))
            out['topics'][name[len(SENSOR_PREFIX) :]] = round(hz, 1) if hz is not None else 0.0
    for name, field, key, digits in FIGURES:
        value = text_number(found.get(name, {}).get(key))
        out[field] = None if value is None else round(value, digits)
    return out
