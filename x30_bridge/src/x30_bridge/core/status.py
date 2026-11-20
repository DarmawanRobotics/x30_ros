"""The stack's status as the handheld's `state` carries it (protocol v1 names, compact parts)."""

# The codes are those of x30_msgs (test_status checks them): core/ stays free of ROS.

from __future__ import annotations

import math
import os

from x30_bridge.core.commands import number

# ControlStatus.FIELD_*: protocol v1 has no 'blind', a blind scan is as unusable as a stale one
FIELD_NAMES = {0: 'off', 1: 'clear', 2: 'limited', 3: 'stop', 4: 'stale', 5: 'stale'}
# NavigationStatus states, as protocol v1 names them
NAV_NAMES = ('NAV_INACTIVE', 'JOINING', 'FOLLOWING', 'APPROACHING', 'DWELLING', 'PAUSED',
             'BLOCKED', 'OFF_PATH', 'GOAL_REACHED', 'FAILED')  # fmt: skip
SEVERITIES = ('normal', 'attention', 'warning', 'critical')  # InspectionFinding
INSPECTION_STATES = ('idle', 'inspecting', 'disabled')  # InspectionStatus
MODE_REMOTE, HEALTH_OK, LOCALIZATION_NONE = 1, 0, 0
MAX_ISSUES = 5
# health finding -> (handheld field, key of its values, digits)
SYSTEM_FIELDS = (
    ('system:temperature', 'temp', 'value', 0),
    ('system:memory', 'ram', 'value', 0),
    ('system:memory', 'ram_used', 'used_gb', 1),
    ('system:memory', 'ram_total', 'total_gb', 1),
    ('system:cpu', 'cpu', 'value', 0),
    ('system:gpu', 'gpu', 'value', 0),
)


def text(value, limit: int) -> str:
    """Return value as text of at most limit characters."""
    return str(value if value is not None else '')[:limit]


def finite(value, digits: int = 2) -> float | None:
    """Return value rounded for JSON, None when it is NaN, infinite or not a number."""
    checked = number(value)
    return None if checked is None else round(checked, digits)


def mode_name(mode: int) -> str:
    """Return 'auto' or 'remote'."""
    return 'remote' if mode == MODE_REMOTE else 'auto'


def field_name(field: int) -> str:
    """Return the protective field: clear, limited, stop, stale or off."""
    return FIELD_NAMES.get(field, 'stale')


def nav_name(state: int) -> str:
    """Return the follower state as protocol v1 names it (NAV_INACTIVE, FOLLOWING, ...)."""
    return NAV_NAMES[state] if 0 <= state < len(NAV_NAMES) else 'FAILED'


def localization_text(state: int, source: str) -> str:
    """Return 'none', or the source of the last correction (tag:<frame>, origin, ...)."""
    return 'none' if state == LOCALIZATION_NONE else (source or 'localized')


def mission_text(state: str, pause_reasons: list, index: int, count: int) -> str:
    """Return a short line about the route: 'paused: operator', 'waypoint 12/240' or ''."""
    if pause_reasons:
        return 'paused: ' + ', '.join(pause_reasons)
    if count and state not in ('NAV_INACTIVE', 'GOAL_REACHED'):
        return f'waypoint {index + 1}/{count}'
    return ''


def trip_summary(distance, moving_sec, speed, average, top, since) -> dict:
    """Return odometry/trip: m, moving s, speed now, average, top (m/s), since (unix s)."""

    def value(x, digits: int) -> float:
        x = number(x)
        return round(x, digits) if x is not None and x >= 0.0 else 0.0

    return {
        'm': value(distance, 1),
        's': int(value(moving_sec, 0)),
        'v': value(speed, 2),
        'avg': value(average, 2),
        'max': value(top, 2),
        'since': int(value(since, 0)),
    }


def finding_name(status_name: str) -> str:
    """Return a health finding's name without the diagnostics prefix ('health: system:cpu')."""
    _, sep, rest = status_name.partition(': ')
    return rest if sep else status_name


def system_figures(findings: dict) -> dict:
    """Return the computer's figures of the health findings ({name: {key: text}}), or None."""
    out = {}
    for name, field, key, digits in SYSTEM_FIELDS:
        try:
            value = float(findings.get(name, {}).get(key, ''))
        except ValueError:
            value = math.nan
        out[field] = round(value, digits) if math.isfinite(value) else None
    return out


def health_summary(level: int, issues: list, findings: dict) -> dict:
    """Return the handheld's health: ok, issues and the computer's load."""
    return {'ok': level == HEALTH_OK, 'issues': [str(i) for i in issues][:MAX_ISSUES],
            **system_figures(findings)}  # fmt: skip


def recorder_summary(status: dict) -> dict:
    """Return the camera recorder's part of `state` (x30_recorder's recorder/status fields)."""
    return {
        'on': bool(status.get('recording')),
        'reason': text(status.get('reason'), 16),
        'session': os.path.basename(text(status.get('session'), 400)),
        'sec': int(number(status.get('duration_sec'), 0.0)),
        'frames': int(number(status.get('thermal_frames'), 0.0)),
        'chunks': int(number(status.get('acoustic_chunks'), 0.0)),
        'videos': int(number(status.get('video_files'), 0.0)),
        'dropped': int(number(status.get('dropped'), 0.0)),
        'streams': [text(s, 60) for s in status.get('streams', [])][:8],
        'free_gb': finite(status.get('free_gb'), 1),
        'msg': text(status.get('message'), 160),
    }


def inspection_summary(status: dict) -> dict:
    """Return the substation inspection's part of `state` (x30_inspection's inspection/status)."""
    state, severity = int(status.get('state', 0)), int(status.get('last_severity', 0))
    return {
        'state': INSPECTION_STATES[state] if 0 <= state < len(INSPECTION_STATES) else 'idle',
        'site': text(status.get('site'), 60),
        'points': int(number(status.get('points'), 0.0)),
        'targets': int(number(status.get('targets'), 0.0)),
        'thermal': bool(status.get('thermal_connected')),
        'acoustic': bool(status.get('acoustic_connected')),
        'current': text(status.get('current'), 60),
        'last': text(status.get('last_point'), 60),
        'severity': SEVERITIES[severity] if 0 <= severity < len(SEVERITIES) else 'critical',
        'ambient': finite(status.get('ambient_c'), 1),
        'msg': text(status.get('message'), 160),
    }
