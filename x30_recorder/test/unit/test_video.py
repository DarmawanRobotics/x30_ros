"""Camera streams: their URL, the password never kept, ffmpeg restarted, real segments written."""

from __future__ import annotations

import json
import shutil
import subprocess

import pytest

from x30_recorder.core.video import StreamRecorder, ffmpeg_command, masked, stream_url
from x30_recorder.utils.streams import stream_specs

SECRET = 'p@ss:w/rd'
OPTIONS = {'ffmpeg': 'ffmpeg', 'segment_sec': 300.0, 'transport': 'tcp', 'restart_sec': 2.0}


class FakeProcess:
    def __init__(self):
        self.returncode = None
        self.stderr = None

    def poll(self):
        return self.returncode


def test_the_url_of_a_channel_and_the_password_masked():
    url = stream_url('192.168.1.64', 554, 'admin', SECRET, 201)
    assert url == 'rtsp://admin:p%40ss%3Aw%2Frd@192.168.1.64:554/Streaming/Channels/201'
    assert SECRET not in masked(f'{url}: 401 Unauthorized ({SECRET})', SECRET)
    assert masked('nothing secret', '') == 'nothing secret'
    command = ffmpeg_command('ffmpeg', url, '/r/v_%Y.mp4', 300.0, 'tcp')
    assert command[command.index('-i') - 2 : command.index('-i')] == ['-rtsp_transport', 'tcp']
    assert command[command.index('-c') : command.index('-c') + 2] == ['-c', 'copy']
    assert '-rtsp_transport' not in ffmpeg_command('ffmpeg', '/clip.mp4', 'p', 10.0, 'tcp')


def test_the_streams_of_a_config():
    class Config:
        host, port, user = '10.0.0.5', 554, 'admin'
        streams = ('visible:101', 'thermal:201', 'broken')
        urls = ('acoustic=rtsp://10.0.0.6:443/live', 'nourl=')

    specs, problems = stream_specs(Config, 'pw')
    assert [name for name, _ in specs] == ['visible', 'thermal', 'acoustic']
    assert specs[1][1].endswith('/Streaming/Channels/201') and len(problems) == 2


def test_a_stopped_ffmpeg_restarts_later_and_later(tmp_path):
    started, logs = [], []

    def popen(command, **_):
        started.append(command)
        return FakeProcess()

    log = lambda level, message: logs.append((level, message))  # noqa: E731
    r = StreamRecorder('visible', 'rtsp://x', SECRET, str(tmp_path), OPTIONS, log)
    r._read = lambda process: None  # no stderr of a fake process
    r.start(0.0, popen)
    assert r.state == 'recording' and len(started) == 1
    r.process.returncode = 1
    r.poll(1.0, popen)
    assert r.state == 'restarting' and len(started) == 1
    r.poll(3.0, popen)  # after 2 s
    assert r.state == 'recording' and len(started) == 2
    r.process.returncode = 1
    r.poll(4.0, popen)
    r.poll(7.0, popen)  # the delay doubled: 4 s
    assert r.state == 'restarting'
    r.poll(8.0, popen)
    assert r.state == 'recording' and r.restarts == 2
    r.poll(70.0, popen)  # a steady minute: quick restarts again
    assert r.delay == OPTIONS['restart_sec']


@pytest.mark.skipif(shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None,
                    reason='no ffmpeg')  # fmt: skip
def test_ffmpeg_copies_a_stream_into_playable_segments(tmp_path):
    clip = str(tmp_path / 'clip.mp4')
    subprocess.run(
        ['ffmpeg', '-nostdin', '-loglevel', 'error', '-f', 'lavfi', '-i',
         'testsrc=duration=6:size=160x120:rate=10', '-c:v', 'mpeg4', '-g', '10', clip],
        check=True, timeout=60,
    )  # fmt: skip
    (tmp_path / 'video').mkdir()
    pattern = str(tmp_path / 'video' / 'visible_%Y%m%d_%H%M%S.mp4')
    command = ffmpeg_command('ffmpeg', clip, pattern, 2.0, 'tcp')
    command.insert(command.index('-i'), '-re')  # the clip at its own pace, as a camera sends it
    run = subprocess.run(command, capture_output=True, text=True, timeout=60, check=False)
    assert run.returncode == 0, run.stderr
    segments = sorted((tmp_path / 'video').glob('visible_*.mp4'))
    assert len(segments) >= 2
    probe = subprocess.run(
        ['ffprobe', '-v', 'error', '-show_entries', 'stream=codec_name,nb_read_frames',
         '-count_frames', '-of', 'json', str(segments[0])],
        capture_output=True, text=True, timeout=60, check=True,
    )  # fmt: skip
    stream = json.loads(probe.stdout)['streams'][0]
    assert stream['codec_name'] == 'mpeg4' and int(stream['nb_read_frames']) > 0
