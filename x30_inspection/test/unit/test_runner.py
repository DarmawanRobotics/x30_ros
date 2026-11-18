"""The point sequence against a fake camera: order, samples, deadline, failures, reports."""

from __future__ import annotations

import dataclasses
from datetime import datetime
import json
import os

from x30_inspection.core import report
from x30_inspection.core.config import CameraConfig, InspectionConfig
from x30_inspection.core.plan import AcousticTarget, Point, Target, ThermalTarget
from x30_inspection.core.runner import Camera, Runner
from x30_inspection.core.thermal import Reading, Severity, worst


class FakeCamera(Camera):
    """Answers with fixed temperatures per box, records every call."""

    def __init__(self, temps=None, fail_point=False, online=True):
        self.temps = temps or {}
        self.fail_point = fail_point
        self.online = online
        self.calls = []

    def connected(self):
        return self.online

    def point(self, pan_deg, tilt_deg, zoom, cancelled):
        self.calls.append(('point', pan_deg, tilt_deg, zoom))
        return 'stalled: no answer' if self.fail_point or not self.online else ''

    def measure(self, box):
        self.calls.append(('measure', box))
        if not self.online:
            return None, 'camera offline'
        t = self.temps.get(box, 35.0)
        return Reading('x', t, 25.0, 1, 2), ''

    def snapshot(self, prefix):
        self.calls.append(('snapshot', prefix))
        return [f'{prefix}_thermal.jpg', f'{prefix}_visible.jpg']

    def park(self):
        self.calls.append(('park',))
        return ''


class Clock:
    """A clock the fake sleep advances."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def runner(fake, **config):
    clock = Clock()
    cfg = dataclasses.replace(InspectionConfig(), **config)
    return Runner(cfg, fake, clock=clock, sleep=clock.sleep), clock


POINT = Point(
    'bay1',
    1.0,
    2.0,
    (
        Target('phases', 30.0, -10.0, 3.0, ThermalTarget(strips=3)),
        Target('ct', -20.0, 0.0, 1.0, ThermalTarget()),
    ),
)


def prefix(target):
    return f'/r/{target.name}'


def test_every_target_in_order_with_evidence():
    camera = FakeCamera()
    run, _ = runner(camera, camera=CameraConfig(park_preset=1))
    result = run.inspect(POINT, prefix)
    kinds = [c[0] for c in camera.calls]
    assert camera.calls[0] == ('point', 30.0, -10.0, 3.0) and kinds[-1] == 'park'
    assert kinds.count('measure') == 3 * 3 + 3  # 3 strips x 3 samples, then the frame x 3
    assert result.state == 'done'
    assert result.targets[0].files == ['/r/phases_thermal.jpg', '/r/phases_visible.jpg']
    assert worst(result.findings) is Severity.NORMAL


def test_hot_strip_is_found():
    hot = {(1 / 3, 0.0, 2 / 3, 1.0): 70.0}
    run, _ = runner(FakeCamera(temps=hot))
    result = run.inspect(POINT, prefix)
    similar = next(f for f in result.findings if f.criterion == 'similar')
    assert similar.label == 'strip2'
    assert similar.severity is Severity.CRITICAL


def test_camera_failure_is_reported_not_raised():
    run, _ = runner(FakeCamera(fail_point=True))
    result = run.inspect(POINT, prefix)
    assert result.state == 'failed'
    assert 'camera: stalled: no answer' in result.message


def test_no_park_preset_leaves_the_camera_where_it_is():
    camera = FakeCamera()
    runner(camera)[0].inspect(POINT, prefix)
    assert 'park' not in [c[0] for c in camera.calls]


def test_offline_camera():
    run, _ = runner(FakeCamera(online=False))
    result = run.inspect(POINT, prefix)
    assert result.state == 'failed'


def test_deadline_skips_the_rest():
    run, _ = runner(FakeCamera(), max_duration_sec=5.0, camera=CameraConfig(settle_sec=4.0))
    result = run.inspect(POINT, prefix)  # settle 4 s + 3 x 2 sample intervals of 0.3 s > 5 s
    assert result.targets[0].measured
    assert result.targets[1].errors == ['skipped: over 5 s']
    assert result.state == 'partial'


def test_cancel_skips_the_targets_and_the_park():
    camera = FakeCamera()
    run, _ = runner(camera, camera=CameraConfig(park_preset=1))
    assert run.inspect(POINT, prefix, cancelled=lambda: True).state == 'failed'
    assert camera.calls == []


def test_cancel_while_measuring_stops_at_the_next_region():
    camera = FakeCamera()
    run, _ = runner(camera, camera=CameraConfig(park_preset=1))
    measures = lambda: sum(c[0] == 'measure' for c in camera.calls)  # noqa: E731
    result = run.inspect(POINT, prefix, cancelled=lambda: measures() >= 3)
    assert measures() == 3  # strip1's samples, then nothing
    assert result.targets[0].errors == ['thermal: cancelled']
    assert result.targets[1].errors == ['cancelled'] and result.state == 'failed'
    assert 'park' not in [c[0] for c in camera.calls]


def test_a_detection_after_a_camera_failure_makes_the_point_partial():
    run, _ = runner(FakeCamera(fail_point=True))

    def detection(target, file_prefix, outcome):
        outcome.measured = True  # what AcousticCheck does after a detection

    run.checks.append(detection)
    both = Target('bushing', 0, 0, acoustic=AcousticTarget(enabled=True))
    result = run.inspect(Point('p', 0, 0, (both,)), prefix)
    assert result.state == 'partial' and 'camera: stalled: no answer' in result.message


def test_no_targets_and_nothing_enabled():
    run, _ = runner(FakeCamera())
    assert run.inspect(Point('empty', 0, 0), prefix).state == 'no_targets'
    idle = Point('idle', 0, 0, (Target('t', 0, 0, thermal=ThermalTarget(enabled=False)),))
    assert run.inspect(idle, prefix).state == 'done'


def test_acoustic_check_runs_without_pointing_the_camera():
    camera = FakeCamera()
    run, _ = runner(camera)
    seen = []

    def check(target, file_prefix, outcome):
        seen.append(target.name)
        outcome.measured = True

    run.checks.append(check)
    only = Target('pd', 0, 0, thermal=ThermalTarget(enabled=False),
                  acoustic=AcousticTarget(enabled=True))  # fmt: skip
    result = run.inspect(Point('p', 0, 0, (only,)), prefix)
    assert seen == ['pd'] and result.state == 'done'
    assert 'point' not in [c[0] for c in camera.calls]


def test_report_folder_json_and_table(tmp_path):
    run, _ = runner(FakeCamera(temps={(1 / 3, 0.0, 2 / 3, 1.0): 70.0}))
    result = run.inspect(POINT, prefix)
    when = datetime(2026, 11, 1, 9, 30, 5)
    folder = report.report_dir(str(tmp_path), when, 'bay1')
    again = report.report_dir(str(tmp_path), when, 'bay1')
    assert folder.endswith('20261101/093005_bay1') and again.endswith('093005_bay1_2')
    data = report.to_dict(result, 'gi_cawang', 1.0, 2.0, when)
    report.write(folder, data)
    saved = json.loads((tmp_path / '20261101' / '093005_bay1' / 'report.json').read_text())
    assert saved['severity'] == 'critical' and saved['state'] == 'done'
    assert saved['targets'][0]['files'] == ['phases_thermal.jpg', 'phases_visible.jpg']
    report.write(again, data)
    rows = (tmp_path / '20261101' / 'findings.csv').read_text().splitlines()
    assert rows[0].startswith('time,site,point,target')
    assert len(rows) == 1 + 2 * len(result.findings)
    assert os.path.basename(report.evidence_prefix(folder, 'ct')) == 'ct'
