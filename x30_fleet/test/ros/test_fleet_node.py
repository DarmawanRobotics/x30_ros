"""fleet_uplink_node against a fake NETRA: telemetry, patrols, commands, uploads."""

from __future__ import annotations

import json
import os

import pytest
from std_msgs.msg import String

from x30_inspection.core import plan as plans
from x30_msgs.msg import InspectionReport, NavigationStatus, RecorderStatus, StopPointEvent
from x30_utils.testing.harness import wait_for

pytest.importorskip('websocket')
pytest.importorskip('requests')
from fleet_rig import TOKEN, Rig, meta_of

ROUTE = b'x,y,yaw_deg,dwell_sec\n0.000,0.000,0.0,0.0\n2.000,0.000,0.0,10.0\n4.000,1.000,90.0,0.0\n'
ROI = {'x': 0.1, 'y': 0.1, 'w': 0.2, 'h': 0.5}
PLAN = {
    'route_id': '7',
    'points': [
        {'id': 12, 'name': 'bay 1 clamps', 'kind': 'thermal', 'stop_index': 0, 'seq': 0,
         'ptz': {'pan_deg': 35.0, 'tilt_deg': -10.0, 'zoom': 4.0}, 'settings': {},
         'targets': [{'key': 'r', 'roi': ROI, 'limit_c': 90}]},
        {'id': 13, 'name': 'bay 1 bushing', 'kind': 'pd', 'stop_index': 0, 'seq': 1,
         'ptz': {'pan_deg': 0.0, 'tilt_deg': 0.0, 'zoom': 1.0}, 'settings': {}, 'targets': []},
    ],
}  # fmt: skip


@pytest.fixture
def rig(ros, tmp_path, request):
    rig = Rig(tmp_path, **getattr(request, 'param', {}))
    yield rig
    rig.close()


def test_hello_with_the_token_and_telemetry_with_the_recorder(rig):
    assert rig.netra.of_type('hello')[0]['robot_id'] == 'x30-test'
    assert rig.netra.headers['Authorization'] == f'Bearer {TOKEN}'
    assert rig.netra.headers['X-Robot-Id'] == 'x30-test'
    first = rig.telemetry(lambda m: True)
    assert first['recorder'] is None and first['substation'] is None and first['estop'] is False
    status = RecorderStatus(recording=True, reason='patrol', session='/data/rec/20261121_0600',
                            duration_sec=12.0, thermal_frames=30, streams=['visible: recording'],
                            free_gb=120.5)  # fmt: skip
    rig.publisher('recorder/status', RecorderStatus).publish(status)
    seen = rig.telemetry(lambda m: m['recorder'] is not None)
    assert seen is not None
    recorder = seen['recorder']
    assert (recorder['on'], recorder['reason'], recorder['session']) == (
        True, 'patrol', '20261121_0600')  # fmt: skip
    assert recorder['frames'] == 30 and recorder['free_gb'] == 120.5


def test_start_patrol_fetches_the_route_and_its_plan_then_starts(rig):
    rig.netra.routes['7'] = ROUTE
    rig.netra.plans['7'] = PLAN
    assert rig.command(1, 'start_patrol', route_id='7', patrol_id=42) == {
        'type': 'ack',
        'id': 1,
        'ok': False,
        'msg': 'no active site: load one first (site/load)',
    }
    rig.site()
    ack = rig.command(2, 'start_patrol', route_id='7', patrol_id=42)
    assert ack['ok'], ack
    assert ack['msg'] == 'patrol started, 2 inspection points'
    assert (rig.sites / 'yard' / 'netra' / '7.csv').read_bytes() == ROUTE
    assert rig.calls == ['load yard/netra/7.csv', 'navigation/start']
    plan = plans.load(rig.plan_file())  # x30_inspection reads it as it is
    (point,) = plan.points
    assert (point.name, point.x, point.y) == ('netra_stop0', 2.0, 0.0)
    assert [t.name for t in point.targets] == ['netra_12', 'netra_13']
    assert point.targets[0].thermal.max_temp_c == 90.0 and point.targets[1].acoustic.enabled
    (started,) = rig.netra.of_type('patrol_event')
    assert (started['event'], started['patrol_id'], started['route_id']) == ('started', 42, '7')
    # the follower's states and stops become NETRA's events
    nav = rig.publisher('navigation/status', NavigationStatus)
    nav.publish(NavigationStatus(state=NavigationStatus.FOLLOWING, index=1, count=3))
    stops = rig.publisher('navigation/stop_point', StopPointEvent)
    stops.publish(StopPointEvent(event=StopPointEvent.ARRIVED, index=1, x=2.0, y=0.0, yaw=0.0))
    assert wait_for(lambda: rig.netra.of_type('stop_event'), 5.0)
    assert rig.netra.of_type('stop_event')[0]['patrol_id'] == 42
    nav.publish(NavigationStatus(state=NavigationStatus.GOAL_REACHED, index=2, count=3))
    assert wait_for(lambda: len(rig.netra.of_type('patrol_event')) == 2, 5.0)
    assert rig.netra.of_type('patrol_event')[1]['event'] == 'finished'


def test_unknown_routes_and_ids_are_refused(rig):
    rig.site()
    assert not rig.command(1, 'start_patrol', route_id='../etc')['ok']
    ack = rig.command(2, 'start_patrol', route_id='99')
    assert not ack['ok'] and ack['msg'].startswith('route download failed')
    assert rig.calls == []


def test_recorder_and_estop_commands(rig):
    assert rig.command(1, 'recorder_start') == {
        'type': 'ack',
        'id': 1,
        'ok': True,
        'msg': 'recorder/start ok',
    }
    assert rig.command(2, 'recorder_stop')['ok']
    assert rig.command(3, 'estop')['ok']
    assert rig.calls == ['recorder/start', 'recorder/stop', 'control/estop/engage']
    assert wait_for(lambda: rig.estops.messages, 5.0)
    assert rig.estops.messages[0].data == 'command center'
    ack = rig.command(4, 'fly')
    assert not ack['ok'] and ack['msg'] == 'unknown command fly'
    ack = rig.command(5, 'dock')  # no docking on this robot yet
    assert not ack['ok']


def test_inspect_point_takes_the_point_where_the_robot_stands(rig):
    rig.site()
    rig.pose(1.5, -2.0, 0.5)
    point = PLAN['points'][0]
    ack = rig.command(1, 'inspect_point', point=point)
    assert ack['ok'] and ack['msg'] == 'inspected'
    assert rig.inspected == [['netra_now']]
    assert plans.load(rig.plan_file()).points == ()  # the point is gone again
    assert not rig.command(2, 'inspect_point', point={'name': 'no id'})['ok']


@pytest.mark.parametrize('rig', [{}], indirect=True)
def test_reports_of_netra_points_and_taught_routes_are_uploaded(rig, tmp_path):
    rig.pose(2.0, 0.0, 1.0)
    assert rig.telemetry(lambda m: m['clock'] is not None, 10.0)  # NETRA's clock is known
    folder = tmp_path / 'reports' / '20261121' / '060000_netra_stop0'
    folder.mkdir(parents=True)
    (folder / 'netra_12_thermal.jpg').write_bytes(b'thermal jpg')
    report = {
        'stamp': 1763700000.0,
        'x': 2.0,
        'y': 0.0,
        'ambient_c': 30.0,
        'targets': [
            {'target': 'netra_12', 'measured': True, 'findings': [],
             'readings': [{'region': 'r', 'max_c': 61.2, 'min_c': 30.1}]},
            {'target': 'bay1_clamps', 'measured': True, 'findings': [],
             'readings': [{'region': 'frame', 'max_c': 40.0, 'min_c': 30.0}]},
        ],
    }  # fmt: skip
    (folder / 'report.json').write_text(json.dumps(report))
    reports = rig.publisher('inspection/report', InspectionReport)
    reports.publish(InspectionReport(site='yard', point='netra_stop0', directory=str(folder)))
    assert wait_for(lambda: rig.uploads('/api/v1/robot/inspections'), 10.0)
    (body,) = rig.uploads('/api/v1/robot/inspections')
    meta = meta_of(body)
    assert meta['point_id'] == 12 and meta['uid'] == 'x30-test.20261121-060000_netra_stop0.12'
    assert meta['thermal'] == [{'key': 'r', 'max_c': 61.2, 'min_c': 30.1}]
    assert meta['pose'] == {'x': 2.0, 'y': 0.0, 'yaw': 1.0}
    assert abs(meta['ts'] - 1763700000.0) < 0.5  # same clock here
    assert b'name="thermal"' in body and b'thermal jpg' in body
    route = tmp_path / 'yard_loop.csv'
    route.write_bytes(ROUTE)
    rig.publisher('route/saved', String).publish(String(data=str(route)))
    assert wait_for(lambda: rig.uploads('/api/v1/robot/routes'), 10.0)
    (body,) = rig.uploads('/api/v1/robot/routes')
    meta = meta_of(body)
    assert (meta['name'], meta['stops'], meta['points']) == ('yard_loop', 1, 3)
    assert b'filename="route.csv"' in body
    spool = tmp_path / 'spool'
    assert wait_for(lambda: all(n.startswith('.') for n in os.listdir(spool)), 5.0)  # all sent


@pytest.mark.parametrize('rig', [{'clock_offset': 5.0}], indirect=True)
def test_measurements_go_onto_netras_clock(rig, tmp_path):
    seen = rig.telemetry(lambda m: m['clock'] is not None, 10.0)
    assert seen is not None and abs(seen['clock']['offset_ms'] - 5000.0) < 500.0
    folder = tmp_path / 'reports' / '20261121' / '060100_netra_stop0'
    folder.mkdir(parents=True)
    report = {
        'stamp': 1000.0,
        'targets': [
            {
                'target': 'netra_12',
                'measured': True,
                'findings': [],
                'readings': [{'region': 'r', 'max_c': 50.0, 'min_c': 30.0}],
            }
        ],
    }
    (folder / 'report.json').write_text(json.dumps(report))  # fmt: skip
    reports = rig.publisher('inspection/report', InspectionReport)
    reports.publish(InspectionReport(directory=str(folder)))
    assert wait_for(lambda: rig.uploads('/api/v1/robot/inspections'), 10.0)
    assert abs(meta_of(rig.uploads('/api/v1/robot/inspections')[0])['ts'] - 1005.0) < 0.5
