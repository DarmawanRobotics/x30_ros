"""The stack's status as the handheld reads it: the codes of x30_msgs, compact parts."""

from __future__ import annotations

from x30_bridge.core import status
from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    InspectionFinding,
    InspectionStatus,
    LocalizationStatus,
    NavigationStatus,
)


def test_the_names_follow_the_codes_of_x30_msgs():
    assert status.mode_name(ControlStatus.MODE_REMOTE) == 'remote'
    assert status.mode_name(ControlStatus.MODE_AUTO) == 'auto'
    assert status.field_name(ControlStatus.FIELD_LIMITED) == 'limited'
    assert status.field_name(ControlStatus.FIELD_BLIND) == 'stale'  # protocol v1 has no blind
    assert status.field_name(ControlStatus.FIELD_OFF) == 'off'
    assert status.nav_name(NavigationStatus.INACTIVE) == 'NAV_INACTIVE'
    assert status.nav_name(NavigationStatus.OFF_PATH) == 'OFF_PATH'
    assert status.nav_name(NavigationStatus.FAILED) == 'FAILED' == status.nav_name(99)
    assert status.localization_text(LocalizationStatus.NONE, 'tag:gate') == 'none'
    assert status.localization_text(LocalizationStatus.LOCALIZED, '') == 'localized'
    assert status.SEVERITIES[InspectionFinding.CRITICAL] == 'critical'
    assert status.INSPECTION_STATES[InspectionStatus.DISABLED] == 'disabled'
    assert status.HEALTH_OK == HealthStatus.OK


def test_a_line_about_the_route():
    assert status.mission_text('PAUSED', ['operator', 'estop'], 3, 10) == 'paused: operator, estop'
    assert status.mission_text('FOLLOWING', [], 11, 240) == 'waypoint 12/240'
    assert status.mission_text('GOAL_REACHED', [], 239, 240) == ''


def test_health_with_the_computers_figures():
    findings = {
        'system:cpu': {'value': '41.6'},
        'system:memory': {'value': '63.2', 'used_gb': '9.81', 'total_gb': '15.5'},
        'system:temperature': {'value': 'nan'},
    }
    health = status.health_summary(1, ['lidar slow', 'tf stale'], findings)
    assert health == {'ok': False, 'issues': ['lidar slow', 'tf stale'], 'cpu': 42.0,
                      'ram': 63.0, 'ram_used': 9.8, 'ram_total': 15.5, 'temp': None,
                      'gpu': None}  # fmt: skip
    assert status.finding_name('health: system:cpu') == 'system:cpu'


def test_the_recorder_and_the_inspection_parts():
    recorder = status.recorder_summary({
        'recording': True, 'reason': 'patrol', 'session': '/data/rec/20261121/101500_patrol',
        'duration_sec': 61.9, 'thermal_frames': 60, 'free_gb': 123.456, 'streams': ['visible: ok'],
    })  # fmt: skip
    assert recorder['session'] == '101500_patrol' and recorder['sec'] == 61
    assert recorder['frames'] == 60 and recorder['free_gb'] == 123.5 and recorder['chunks'] == 0
    inspecting = {'state': 1, 'last_severity': 3, 'points': 4, 'ambient_c': float('nan')}
    inspection = status.inspection_summary(inspecting)
    assert inspection['state'] == 'inspecting' and inspection['severity'] == 'critical'
    assert inspection['points'] == 4 and inspection['ambient'] is None
    trip = status.trip_summary(12.345, 61.0, float('nan'), 0.4, 0.8, 1.7e9)
    assert trip == {'m': 12.3, 's': 61, 'v': 0.0, 'avg': 0.4, 'max': 0.8, 'since': 1700000000}
