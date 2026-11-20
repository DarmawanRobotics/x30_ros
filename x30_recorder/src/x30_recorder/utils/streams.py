"""The video streams of a session: the camera's channels and any other input, one ffmpeg each."""

from __future__ import annotations

import os

from x30_recorder.core.video import StreamRecorder, stream_url


def stream_specs(config, password: str) -> tuple[list, list[str]]:
    """Return ([(name, url)], problems) of a VideoConfig and the camera's password."""
    specs, problems = [], []
    for item in config.streams:
        name, _, channel = item.partition(':')
        if not name or not channel.isdigit():
            problems.append(f'video stream {item!r} is not name:channel')
            continue
        url = stream_url(config.host, config.port, config.user, password, int(channel))
        specs.append((name, url))
    for item in config.urls:
        name, _, url = item.partition('=')
        if not name or not url:
            problems.append(f'video url {item!r} is not name=url')
            continue
        specs.append((name, url))
    return specs, problems


class Streams:
    """The StreamRecorders of the running session."""

    def __init__(self, config, log):
        self.config, self.log = config, log
        self.password = os.environ.get(config.password_env, '') if config.password_env else ''
        self.specs, self.problems = (
            stream_specs(config, self.password) if config.enabled else ([], [])
        )
        self.recorders: list[StreamRecorder] = []

    def start(self, directory: str, now: float):
        """Start every stream into directory."""
        c = self.config
        options = {
            'ffmpeg': c.ffmpeg,
            'segment_sec': c.segment_sec,
            'transport': c.transport,
            'restart_sec': c.restart_sec,
        }
        self.recorders = [
            StreamRecorder(name, url, self.password, directory, options, self.log)
            for name, url in self.specs
        ]
        for recorder in self.recorders:
            recorder.start(now)

    def poll(self, now: float):
        """Restart what stopped."""
        for recorder in self.recorders:
            recorder.poll(now)

    def stop(self):
        """Stop every stream (each closes its last segment)."""
        recorders, self.recorders = self.recorders, []
        for recorder in recorders:
            recorder.stop()

    def states(self) -> list[str]:
        """Return name: state of every stream."""
        return [f'{r.name}: {r.state}' for r in self.recorders]
