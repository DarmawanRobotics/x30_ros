"""The patrol a NETRA start opens: laps, the end, alerts, stop points."""

from __future__ import annotations

from x30_fleet.core.patrol import Patrol


def kinds(events):
    return [(e['type'], e.get('event') or e.get('state')) for e in events]


def test_a_patrol_counts_laps_and_finishes():
    patrol = Patrol()
    assert kinds(patrol.start('p1', 'r7')) == [('patrol_event', 'started')]
    assert patrol.context()['patrol_id'] == 'p1'
    assert patrol.navigation('FOLLOWING', '', 10, 100, None) == []
    assert patrol.navigation('FOLLOWING', '', 90, 100, None) == []
    assert kinds(patrol.navigation('FOLLOWING', '', 2, 100, None)) == [('patrol_event', 'lap')]
    assert patrol.mission() == 'lap 2'
    events = patrol.navigation('GOAL_REACHED', 'done', 99, 100, None)
    assert kinds(events) == [('patrol_event', 'finished')]
    assert events[0]['reason'] == 'goal_reached' and patrol.current is None


def test_alerts_on_state_changes_only():
    patrol = Patrol()
    pose = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    events = patrol.navigation('BLOCKED', 'no progress', 5, 50, pose)
    assert kinds(events) == [('alert', 'BLOCKED')] and events[0]['pose'] == pose
    assert patrol.navigation('BLOCKED', 'no progress', 5, 50, pose) == []
    assert kinds(patrol.navigation('FAILED', 'x', 5, 50, pose)) == [('alert', 'FAILED')]
    assert patrol.navigation('NAV_INACTIVE', '', 0, 0, pose) == []  # no patrol: no finish


def test_estop_and_stop_points():
    patrol = Patrol()
    alert = patrol.estop(['handheld estop'], None)
    assert alert['state'] == 'ESTOP' and 'handheld estop' in alert['message']
    event = patrol.stop_point('arrived', 4, '', 1.0, 2.0, 90.0)
    assert event['type'] == 'stop_event' and event['yaw_deg'] == 90.0
    assert patrol.stop_index == 4
    patrol.stop_point('departed', 4, '', 1.0, 2.0, 90.0)
    assert patrol.stop_index is None
