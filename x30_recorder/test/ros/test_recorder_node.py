"""In-process recorder_node: a patrol session of raw thermal frames, manual with acoustic data."""

from __future__ import annotations

import itertools
import json
import math
from pathlib import Path
import shutil

from geometry_msgs.msg import PoseWithCovarianceStamped
from hertzino_msgs.msg import DetectionResult, RawFrame, Spectrum
from hikmicro_msgs.msg import PtzState
import numpy as np
import pytest
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from x30_msgs.msg import NavigationStatus, RecorderStatus, SiteStatus
from x30_recorder.recorder_node import RecorderNode
from x30_utils.io import read_thermal
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

NAMES = itertools.count()
SECRET = 's3cr3t!pw'


def thermal_image(t: np.ndarray, stamp: rospy.Time) -> Image:
    msg = Image(height=t.shape[0], width=t.shape[1], encoding='32FC1', step=4 * t.shape[1])
    msg.data = t.astype('<f4').tobytes()
    msg.header.stamp, msg.header.frame_id = stamp, 'hikmicro_thermal_optical_frame'
    return msg


class Rig:
    """The recorder in this process, publishers of what it records, its status."""

    def __init__(self, tmp_path: Path, **params):
        self.ns = f'/recorder_test_{next(NAMES)}'
        self.root = tmp_path / 'recordings'
        defaults = {
            'recordings_dir': str(self.root),
            'video.enabled': False,
            'patrol.stop_after_sec': 0.5,
            'acoustic.chunk_sec': 1.0,
            'disk.min_free_gb': 0.0,
            'disk.stop_free_gb': 0.0,
        }
        self.node = RecorderNode(param_ns=f'{self.ns}/node', overrides={**defaults, **params})
        latched = {'queue_size': 1, 'latch': True}
        self.pubs = {
            'thermal': rospy.Publisher('/hikmicro/thermal/temperature', Image, queue_size=10),
            'ptz': rospy.Publisher('/hikmicro/ptz/state', PtzState, **latched),
            'pose': rospy.Publisher('/localization/pose', PoseWithCovarianceStamped, queue_size=1),
            'nav': rospy.Publisher('/navigation/status', NavigationStatus, **latched),
            'site': rospy.Publisher('/site/status', SiteStatus, **latched),
            'spectrum': rospy.Publisher('/hertzino/spectrum', Spectrum, queue_size=10),
            'prpd': rospy.Publisher('/hertzino/prpd', RawFrame, queue_size=10),
            'detection': rospy.Publisher('/hertzino/detection', DetectionResult, queue_size=10),
        }
        self.status = Collector(RecorderStatus, 'recorder/status')
        assert connected(*self.pubs.values(), self.status)

    def call(self, name: str):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, Trigger)()

    def recording(self, reason: str):
        return wait_for(lambda: self.status.last and self.status.last.reason == reason, 5.0)

    def close(self):
        self.node.close()
        self.status.close()
        for pub in self.pubs.values():
            pub.unregister()
        self.node.destroy_node()
        delete_params(self.ns)


@pytest.fixture
def make_rig(ros, tmp_path):
    rigs = []

    def make(**params):
        rigs.append(Rig(tmp_path, **params))
        return rigs[-1]

    yield make
    for rig in rigs:
        rig.close()


def test_a_patrol_records_raw_thermal_frames_with_pose_and_camera(make_rig):
    rig = make_rig()
    rig.pubs['site'].publish(SiteStatus(site='gardu'))
    rig.pubs['ptz'].publish(PtzState(connected=True, pan=120.5, tilt=-10.0, zoom=4.0))
    rig.pubs['nav'].publish(NavigationStatus(state=NavigationStatus.FOLLOWING))
    assert rig.recording('patrol')
    session = Path(rig.status.last.session)
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x, pose.pose.pose.position.y = 1.5, -2.0
    pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(0.25), math.cos(0.25)
    rig.pubs['pose'].publish(pose)
    assert wait_for(lambda: rig.node.pose is not None, 3.0)
    frames = [np.full((2, 3), 30.0 + i, dtype=np.float32) for i in range(3)]
    for i, t in enumerate(frames):
        rig.pubs['thermal'].publish(thermal_image(t, rospy.Time.now() + rospy.Duration(0.1 * i)))
    assert wait_for(lambda: len(list((session / 'thermal').glob('*.npz'))) == 3, 5.0)
    first = read_thermal(str(sorted((session / 'thermal').glob('*.npz'))[0]))
    assert np.array_equal(first['temperature'], frames[0])
    assert first['pose'] == pytest.approx([1.5, -2.0, 0.5])
    assert tuple(first['ptz']) == (120.5, -10.0, 4.0)
    assert first['meta']['site'] == 'gardu'
    rig.pubs['nav'].publish(NavigationStatus(state=NavigationStatus.GOAL_REACHED))
    assert wait_for(lambda: not rig.status.last.recording, 5.0)
    assert wait_for(lambda: json.loads((session / 'session.json').read_text())['stopped'], 5.0)
    record = json.loads((session / 'session.json').read_text())
    assert (record['reason'], record['thermal_frames'], record['site']) == ('patrol', 3, 'gardu')


def test_manual_recording_keeps_the_acoustic_data(make_rig):
    rig = make_rig()
    started = rig.call('/recorder/start')
    assert started.success and rig.recording('manual')
    session = Path(started.message)
    for i in range(3):
        msg = Spectrum(bin_hz=5.0, magnitude=list(range(i, i + 8)), band_peak=7 + i)
        msg.header.stamp = rospy.Time.now()
        rig.pubs['spectrum'].publish(msg)
        rig.pubs['prpd'].publish(RawFrame(stream='prpdData', data=bytes([i, 255, 0])))
    rig.pubs['detection'].publish(DetectionResult(type='corona', abnormal=True, intensity=42))
    assert wait_for((session / 'acoustic' / 'detections.jsonl').exists, 5.0)
    stopped = rig.call('/recorder/stop')
    assert stopped.success and stopped.message == str(session)
    assert wait_for(lambda: json.loads((session / 'session.json').read_text())['stopped'], 5.0)
    spectra = list((session / 'acoustic').glob('spectrum_*.npz'))
    assert spectra and list((session / 'acoustic').glob('prpd_*.npz'))
    with np.load(str(spectra[0]), allow_pickle=False) as data:
        assert data['magnitude'].dtype == np.int16 and len(data['stamp']) >= 1
    line = json.loads((session / 'acoustic' / 'detections.jsonl').read_text().splitlines()[0])
    assert line['type'] == 'corona' and line['intensity'] == 42
    assert wait_for(lambda: not rig.status.last.recording, 3.0)


def test_no_recording_on_a_full_disk(make_rig):
    rig = make_rig(**{'disk.min_free_gb': 9999.0, 'disk.stop_free_gb': 9999.0})
    refused = rig.call('/recorder/start')
    assert not refused.success and 'GB free' in refused.message
    assert wait_for(lambda: rig.status.last and 'GB free' in rig.status.last.message, 3.0)
    assert not list(rig.root.glob('*/*'))


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='no ffmpeg')
def test_a_camera_stream_that_fails_is_retried_and_its_password_never_kept(make_rig, monkeypatch):
    monkeypatch.setenv('HIKMICRO_PASSWORD', SECRET)
    rig = make_rig(**{
        'video.enabled': True, 'video.host': '127.0.0.1', 'video.port': 9,
        'video.streams': ['visible:101'],
    })  # fmt: skip
    session = Path(rig.call('/recorder/start').message)
    log = session / 'video' / 'visible.log'
    assert wait_for(lambda: log.exists() and log.read_text().strip(), 10.0)
    assert wait_for(
        lambda: rig.status.last and 'visible: restarting' in rig.status.last.streams, 10.0
    )
    assert SECRET not in log.read_text() and '***' in log.read_text()
    rig.call('/recorder/stop')
    assert all(SECRET not in p.read_text() for p in session.rglob('*.json'))
