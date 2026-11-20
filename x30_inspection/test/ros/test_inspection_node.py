"""inspection_node against faked camera drivers: stops, reports, evidence, teaching, failures."""

from __future__ import annotations

import itertools
import json
from pathlib import Path
import threading

from fake_cameras import FRAME, FakeHertzino, FakeHikmicro
import numpy as np
import pytest
import rospy
from std_srvs.srv import Trigger
import yaml

from x30_inspection.inspection_node import InspectionNode
from x30_msgs.msg import (
    InspectionFinding,
    InspectionReport,
    InspectionStatus,
    SiteStatus,
    StopPointEvent,
)
from x30_msgs.srv import AddTarget
from x30_utils.io import read_thermal
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

NAMES = itertools.count()
SITE = 'lobby'
PHASES = {'name': 'phases', 'pan_deg': 30.0, 'tilt_deg': -10.0, 'zoom': 3.0}
BUSHING = {'name': 'bushing', 'pan_deg': 0.0, 'tilt_deg': 0.0}
PLAN = {
    'points': [
        {
            'name': 'bay1',
            'x': 1.0,
            'y': 2.0,
            'targets': [
                {**PHASES, 'thermal': {'strips': 3}},
                {**BUSHING, 'thermal': {'enabled': False}, 'acoustic': {'enabled': True}},
            ],
        }
    ]
}
MIDDLE = (1 / 3, 0.0, 2 / 3, 1.0)  # strip2 of three


class Rig:
    """The node with its own camera namespaces, the fakes, a site and a stop publisher."""

    def __init__(self, tmp: Path, temps=None, cameras: bool = True, **overrides):
        n = next(NAMES)
        self.ns = f'/inspection_test_{n}'
        self.reports = tmp / 'reports'
        (tmp / SITE).mkdir(exist_ok=True)
        (tmp / SITE / 'inspection.yaml').write_text(yaml.safe_dump(PLAN))
        hik, hz = f'{self.ns}/hikmicro', f'{self.ns}/hertzino'
        self.camera = FakeHikmicro(hik, temps) if cameras else None
        self.acoustic = FakeHertzino(hz, status=4, intensity=40) if cameras else None  # surface
        values = {
            'sites_dir': str(tmp),
            'reports_dir': str(self.reports),
            'camera.namespace': hik,
            'camera.settle_sec': 0.0,
            'camera.park_preset': 2,
            'thermal.samples': 1,
            'acoustic.namespace': hz,
            'acoustic.duration_sec': 1.0,
            **overrides,
        }
        self.node = InspectionNode(param_ns=f'{self.ns}/inspection_node', overrides=values)
        self.reports_msgs = Collector(InspectionReport, 'inspection/report')
        self.status = Collector(InspectionStatus, 'inspection/status')
        self.site_pub = rospy.Publisher('site/status', SiteStatus, queue_size=1, latch=True)
        self.stop_pub = rospy.Publisher('navigation/stop_point', StopPointEvent, queue_size=5)
        assert connected(self.site_pub, self.stop_pub)
        self.site_pub.publish(SiteStatus(site=SITE))
        assert wait_for(lambda: self.node.site == SITE, 5.0)

    def stop_at(self, x: float, y: float):
        self.stop_pub.publish(StopPointEvent(event=StopPointEvent.ARRIVED, x=x, y=y))
        assert wait_for(lambda: self.node.stop is not None and self.node.stop[0] == x, 5.0)

    def call(self, srv_type, service: str, **request):
        rospy.wait_for_service(service, 5.0)
        return rospy.ServiceProxy(service, srv_type)(**request)

    def close(self):
        for handle in (self.reports_msgs, self.status):
            handle.close()
        self.site_pub.unregister()
        self.stop_pub.unregister()
        self.node.run_shutdown_hooks()
        self.node.destroy_node()
        for fake in (self.camera, self.acoustic):
            if fake is not None:
                fake.close()
        delete_params(self.ns)


@pytest.fixture
def make(ros, tmp_path):
    made = []

    def factory(**kwargs):
        made.append(Rig(tmp_path, **kwargs))
        return made[-1]

    yield factory
    for rig in made:
        rig.close()


def test_inspects_the_point_where_the_robot_stopped(make):
    """Thermal strips and partial discharge at the stop; report, evidence, the camera parked."""
    rig = make(temps={MIDDLE: 70.0})
    rig.stop_at(1.2, 2.1)
    answer = rig.call(Trigger, 'inspection/inspect')
    assert answer.success and answer.message == 'bay1: 2/2 targets measured', answer.message
    assert rig.camera.moves == [(30.0, -10.0, 3.0)] and rig.camera.presets == [2]
    assert all(normalized for _, normalized in rig.camera.measured)
    assert rig.acoustic.goals[0].duration == pytest.approx(1.0)
    assert rig.acoustic.goals[0].fetch_photos and not rig.acoustic.goals[0].keep_running
    assert rig.reports_msgs.wait(1, 5.0)
    msg = rig.reports_msgs.last
    assert (msg.point, msg.state, msg.site) == ('bay1', InspectionReport.DONE, SITE)
    assert msg.severity == InspectionFinding.CRITICAL  # strip2 35 degC over the others
    similar = next(f for f in msg.findings if f.criterion == 'similar')
    assert similar.label == 'strip2' and similar.value == pytest.approx(70.0)
    discharge = next(f for f in msg.findings if f.sensor == 'acoustic')
    assert discharge.label == 'surface' and discharge.severity == InspectionFinding.WARNING
    folder = Path(msg.directory)
    files = sorted(p.name for p in folder.iterdir())
    assert files == ['bushing_prpd.jpg', 'bushing_soundmap.jpg', 'phases_thermal.jpg',
                     'phases_thermal.npz', 'phases_visible.jpg', 'report.json']  # fmt: skip
    raw = read_thermal(str(folder / 'phases_thermal.npz'))  # the frame once the camera arrived
    assert np.array_equal(raw['temperature'], FRAME) and tuple(raw['ptz']) == (30.0, -10.0, 3.0)
    assert raw['meta'] == {'frame_id': 'thermal_optical', 'target': 'phases'}
    saved = json.loads((folder / 'report.json').read_text())
    assert saved['severity'] == 'critical' and saved['x'] == pytest.approx(1.2)
    assert saved['ambient_c'] == 30.0 and saved['stamp'] > 0
    phases = saved['targets'][0]['readings']  # every strip as measured, for NETRA
    assert [r['region'] for r in phases] == ['strip1', 'strip2', 'strip3']
    assert phases[1]['max_c'] == pytest.approx(70.0) and phases[0]['max_c'] == pytest.approx(35.0)
    assert (folder.parent / 'findings.csv').read_text().count('\n') == 1 + len(msg.findings)
    assert rig.status.wait_until(lambda s: s.last_point == 'bay1' and s.last_severity == 3, 5.0)


def test_no_raw_thermal_frame_from_before_the_camera_arrived(make):
    rig = make(**{'camera.raw_wait_sec': 0.3})
    rig.camera.frame_age = 60.0  # only frames taken a minute ago
    rig.stop_at(1.0, 2.0)
    assert rig.call(Trigger, 'inspection/inspect').success
    assert rig.reports_msgs.wait(1, 5.0)
    folder = Path(rig.reports_msgs.last.directory)
    assert (folder / 'phases_thermal.jpg').exists() and not list(folder.glob('*.npz'))


def test_a_stop_without_a_point_answers_at_once(make):
    rig = make()
    rig.stop_at(9.0, 9.0)
    answer = rig.call(Trigger, 'inspection/inspect')
    assert answer.success and 'no inspection point within 1.5 m' in answer.message
    assert not rig.camera.moves and not rig.reports_msgs.messages


def test_missing_cameras_are_reported_not_raised(make):
    """No drivers: the point fails with the reasons, the node answers and stays up."""
    rig = make(cameras=False)
    rig.stop_at(1.0, 2.0)
    answer = rig.call(Trigger, 'inspection/inspect')
    assert not answer.success
    assert 'camera: ptz/move_to not available' in answer.message
    assert 'acoustic: detect not available' in answer.message
    assert rig.reports_msgs.wait(1, 5.0) and rig.reports_msgs.last.state == InspectionReport.FAILED
    offline = lambda s: not (s.thermal_connected or s.acoustic_connected)  # noqa: E731
    assert rig.status.wait_until(offline, 5.0)


def test_a_stalled_camera_skips_the_thermal_part(make):
    rig = make()
    rig.camera.stall = True
    rig.stop_at(1.0, 2.0)
    answer = rig.call(Trigger, 'inspection/inspect')
    assert answer.success  # the acoustic target was measured: partial
    assert 'camera: ptz/move_to stalled: the PTZ stopped short' in answer.message
    assert (
        rig.reports_msgs.wait(1, 5.0) and rig.reports_msgs.last.state == InspectionReport.PARTIAL
    )


def test_one_inspection_at_a_time(make):
    rig = make(**{'acoustic.duration_sec': 2.0})
    rig.stop_at(1.0, 2.0)
    first = []
    thread = threading.Thread(target=lambda: first.append(rig.call(Trigger, 'inspection/inspect')))
    thread.start()
    assert rig.status.wait_until(lambda s: s.state == InspectionStatus.INSPECTING, 5.0)
    second = rig.call(Trigger, 'inspection/inspect')
    assert not second.success and second.message == 'an inspection is running'
    taught = rig.call(AddTarget, 'inspection/add_target', name='late', thermal=True)
    assert not taught.success and 'an inspection is running' in taught.message
    thread.join(15.0)
    assert first and first[0].success


def test_status_counts_the_plan_and_sees_the_cameras(make):
    rig = make()
    ready = lambda s: s.thermal_connected and s.acoustic_connected  # noqa: E731
    assert rig.status.wait_until(lambda s: ready(s) and s.points == 1 and s.targets == 2, 6.0)
    assert rig.status.last.state == InspectionStatus.IDLE and rig.status.last.site == SITE
    assert rig.status.last.ambient_c == pytest.approx(30.0)


def test_add_target_teaches_from_the_cameras_view(make, tmp_path):
    rig = make()
    with rig.camera.lock:
        rig.camera.pose = [123.456, -12.5, 8.0]
    rig.stop_at(5.0, 5.0)
    assert wait_for(lambda: (rig.node.camera.pose() or (0,))[0] == pytest.approx(123.456), 5.0)
    answer = rig.call(AddTarget, 'inspection/add_target', name='ct', thermal=True, strips=3)
    assert answer.success and (answer.point, answer.targets) == ('ct', 1), answer.message
    saved = yaml.safe_load((tmp_path / SITE / 'inspection.yaml').read_text())
    point = next(p for p in saved['points'] if p['name'] == 'ct')
    assert (point['x'], point['y']) == (5.0, 5.0)
    target = point['targets'][0]
    assert (target['pan_deg'], target['tilt_deg'], target['zoom']) == (123.46, -12.5, 8.0)
    assert target['thermal']['strips'] == 3
    again = rig.call(AddTarget, 'inspection/add_target', name='ct', thermal=True)
    assert not again.success and 'already' in again.message
    assert (tmp_path / SITE / 'inspection.yaml.bak').is_file()
