"""A recording session on disk: <recordings_dir>/<YYYYMMDD>/<HHMMSS>_<reason>/ and its record."""

# session.json says what the session is (reason, start and end, site, what was recorded) and is
# rewritten as it grows: a session cut by a power loss still says what it holds. Its folders:
# thermal/ (a .npz per frame), video/ (.mp4 segments and each stream's ffmpeg log), acoustic/
# (.npz chunks and detections.jsonl).

from __future__ import annotations

from datetime import datetime
import os

from x30_utils.io import write_json_atomic

REASONS = ('patrol', 'manual')


def stamp_name(stamp: float) -> str:
    """Return a file name stem of a time: <YYYYMMDD>_<HHMMSS>_<ms> (local time)."""
    when = datetime.fromtimestamp(stamp)
    return when.strftime('%Y%m%d_%H%M%S_') + f'{when.microsecond // 1000:03d}'


class Session:
    """One recording: its folder, its counters and its session.json."""

    def __init__(self, root: str, reason: str, started: float, info: dict | None = None):
        if reason not in REASONS:
            raise ValueError(f'reason {reason!r} is not one of {REASONS}')
        when = datetime.fromtimestamp(started)
        day = os.path.join(os.path.expanduser(root), when.strftime('%Y%m%d'))
        name = when.strftime('%H%M%S') + '_' + reason
        path, n = os.path.join(day, name), 1
        while os.path.exists(path):  # two sessions started within one second
            path, n = os.path.join(day, f'{name}-{n}'), n + 1
        os.makedirs(path)
        self.path, self.reason, self.started = path, reason, started
        self.info = dict(info or {})
        self.counts = {'thermal_frames': 0, 'acoustic_chunks': 0, 'video_files': 0, 'dropped': 0}
        self.stopped: float | None = None
        self.write()

    def file(self, kind: str, name: str) -> str:
        """Return the path of a file of the session in its kind's folder (made when missing)."""
        directory = os.path.join(self.path, kind)
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, name)

    def count(self, key: str, n: int = 1):
        """Count n more of what the session holds."""
        self.counts[key] = self.counts.get(key, 0) + n

    def duration(self, now: float) -> float:
        """Return how long the session ran (s) until now or its stop."""
        return max(0.0, (self.stopped if self.stopped is not None else now) - self.started)

    def stop(self, when: float):
        """End the session and write its record."""
        self.stopped = when
        self.write()

    def write(self):
        """Rewrite session.json from what is known now."""
        record = {
            'reason': self.reason,
            'started': datetime.fromtimestamp(self.started).isoformat(timespec='seconds'),
            'stopped': None
            if self.stopped is None
            else datetime.fromtimestamp(self.stopped).isoformat(timespec='seconds'),
            **self.counts,
            **self.info,
        }
        write_json_atomic(os.path.join(self.path, 'session.json'), record, backup=False)
