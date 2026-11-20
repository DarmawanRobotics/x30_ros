"""NETRA's substation plan of a route as x30_inspection's; its reports as NETRA's uploads."""

from __future__ import annotations

import re

from x30_fleet.core import substation
from x30_inspection.core import plan as plans

ROUTE = (
    'x,y,yaw_deg,dwell_sec\n'
    '0.000,0.000,0.0,0.0\n'
    '2.000,0.000,0.0,10.0\n'
    '4.000,1.000,90.0,0.0\n'
    '6.000,2.000,90.0,15.0\n'
)
NETRA_UID = re.compile(r'^[A-Za-z0-9._-]{1,128}$')


def netra_point(pid, kind, stop, seq=0, **extra) -> dict:
    """Return a point as NETRA's GET /api/v1/robot/routes/{id}/inspection lists it."""
    return {
        'id': pid,
        'name': f'point {pid}',
        'kind': kind,
        'stop_index': stop,
        'seq': seq,
        'ptz': {'pan_deg': 30.0, 'tilt_deg': -10.0, 'zoom': 4.0},
        'settings': {},
        'targets': [],
        **extra,
    }


def test_the_stops_are_the_rows_with_a_dwell():
    assert substation.route_stops(ROUTE) == [(2.0, 0.0), (6.0, 2.0)]
    assert substation.route_stops('# a comment\nbad,row\n1,2\n1,2,0\n') == []


def test_netra_points_become_targets_at_their_stops():
    targets = [
        {'key': 'r', 'roi': {'x': 0.1, 'y': 0.2, 'w': 0.2, 'h': 0.3}, 'limit_c': 90.0},
        {'key': 's', 'roi': {'x': 0.4, 'y': 0.2, 'w': 0.2, 'h': 0.3}, 'limit_c': 80.0},
        {'key': 't', 'limit_c': None},  # no region: the whole frame
    ]
    plan = {
        'route_id': '7',
        'points': [
            netra_point(12, 'thermal', 1, targets=targets),
            netra_point(
                14, 'thermal', 0, seq=1, ptz={'pan_deg': 999, 'tilt_deg': -120, 'zoom': 0}
            ),
            netra_point(13, 'pd', 0),
        ],
    }
    out, skipped = substation.to_plan(plan, substation.route_stops(ROUTE))
    assert skipped == []
    first, second = out['points']
    assert (first['name'], first['x'], first['y']) == ('netra_stop0', 2.0, 0.0)
    assert (second['name'], second['x'], second['y']) == ('netra_stop1', 6.0, 2.0)
    assert [t['name'] for t in first['targets']] == ['netra_13', 'netra_14']  # by seq
    pd, clamped = first['targets']
    assert pd['acoustic'] == {'enabled': True} and not pd['thermal']['enabled']
    assert (clamped['pan_deg'], clamped['tilt_deg'], clamped['zoom']) == (360.0, -90.0, 1.0)
    thermal = second['targets'][0]['thermal']
    assert thermal['enabled'] and thermal['max_temp_c'] == 80.0
    assert thermal['regions'] == [
        {'name': 'r', 'box': [0.1, 0.2, 0.3, 0.5]},
        {'name': 's', 'box': [0.4, 0.2, 0.6, 0.5]},
        {'name': 't', 'box': [0.0, 0.0, 1.0, 1.0]},
    ]
    assert plans.parse(out).targets == 3  # x30_inspection takes it as it is


def test_limits_beyond_x30_inspection_and_bad_regions():
    targets = [
        {'key': 'a', 'roi': {'x': 0.1, 'y': 0.1, 'w': 0.0, 'h': 0.3}},  # empty box
        {'key': 'Phase R', 'roi': {'x': 0.1, 'y': 0.1, 'w': 0.2, 'h': 0.3}},  # not a name
        {'key': 'b', 'roi': {'x': 0.9, 'y': 0.9, 'w': 0.5, 'h': 0.5}, 'limit_c': 1200.0},
    ]
    out, _ = substation.to_plan({'points': [netra_point(3, 'thermal', 0, targets=targets)]},
                                [(1.0, 1.0)])  # fmt: skip
    thermal = out['points'][0]['targets'][0]['thermal']
    assert thermal['regions'] == [{'name': 'b', 'box': [0.9, 0.9, 1.0, 1.0]}]
    assert thermal['max_temp_c'] == 500.0
    assert plans.parse(out).targets == 1


def test_points_off_the_route_are_left_out():
    plan = {
        'points': [
            netra_point(1, 'thermal', 5),  # the route has one stop
            netra_point(2, 'visual', 0),
            {'kind': 'pd', 'stop_index': 0},  # no id
            'junk',
        ]
    }
    out, skipped = substation.to_plan(plan, [(0.0, 0.0)])
    assert out == {'points': []} and len(skipped) == 3
    assert substation.to_plan({}, [(0.0, 0.0)]) == ({'points': []}, [])


def test_target_names_lead_back_to_netra_points():
    assert substation.point_id('netra_12') == 12
    assert substation.point_id('bay1_clamps') is None and substation.point_id('') is None


def report_dir(tmp_path):
    folder = tmp_path / 'reports' / '20261121' / '055800_netra_stop0'
    folder.mkdir(parents=True)
    for name in ('netra_12_thermal.jpg', 'netra_12_visible.jpg', 'netra_13_soundmap.jpg'):
        (folder / name).write_bytes(b'jpg')
    return folder


REPORT = {
    'stamp': 1763700000.5,
    'x': 2.0,
    'y': 0.0,
    'ambient_c': 30.0,
    'targets': [
        {
            'target': 'netra_12',
            'measured': True,
            'readings': [
                {'region': 'r', 'max_c': 61.2, 'min_c': 30.1},
                {'region': 's', 'max_c': 40.0, 'min_c': 29.0},
            ],
            'findings': [],
        },
        {
            'target': 'netra_13',
            'measured': True,
            'readings': [],
            'findings': [{'sensor': 'acoustic', 'label': 'corona', 'value': 250.0}],
        },
        {
            'target': 'bay1_clamps',  # the site's own point, not NETRA's
            'measured': True,
            'readings': [{'region': 'frame', 'max_c': 50.0, 'min_c': 30.0}],
            'findings': [],
        },
        {'target': 'netra_15', 'measured': False, 'readings': [], 'findings': []},
    ],
}


def test_reports_of_netra_points_become_uploads(tmp_path):
    folder = report_dir(tmp_path)
    (thermal, thermal_files), (pd, pd_files) = substation.uploads(
        REPORT, str(folder), 'x30-01', 7, 1.5
    )
    assert thermal == {
        'uid': 'x30-01.20261121-055800_netra_stop0.12',
        'point_id': 12,
        'ts': 1763700000.5,
        'patrol_id': 7,
        'pose': {'x': 2.0, 'y': 0.0, 'yaw': 1.5},
        'ambient_c': 30.0,
        'thermal': [
            {'key': 'r', 'max_c': 61.2, 'min_c': 30.1},
            {'key': 's', 'max_c': 40.0, 'min_c': 29.0},
        ],
    }
    assert NETRA_UID.match(thermal['uid']) and NETRA_UID.match(pd['uid'])
    assert sorted(thermal_files) == ['thermal', 'visual']
    assert thermal_files['thermal'] == str(folder / 'netra_12_thermal.jpg')
    assert pd['pd'] == {'db': 200.0, 'detected': True, 'kind': 'corona'}  # NETRA's range
    assert list(pd_files) == ['acoustic'] and 'thermal' not in pd


def test_noise_is_no_discharge_and_nothing_measured_no_upload(tmp_path):
    report = {'stamp': 1.0, 'targets': [
        {'target': 'netra_13', 'measured': True, 'readings': [],
         'findings': [{'sensor': 'acoustic', 'label': 'noise', 'value': 12.0}]},
        {'target': 'netra_14', 'measured': True, 'readings': [], 'findings': []},
    ]}  # fmt: skip
    ((meta, files),) = substation.uploads(report, str(tmp_path), 'x30-01', None, 0.0)
    assert meta['pd'] == {'db': 12.0, 'detected': False, 'kind': 'noise'}
    assert meta['patrol_id'] is None and 'ambient_c' not in meta and files == {}
