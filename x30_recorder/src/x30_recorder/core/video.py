"""A camera stream copied by ffmpeg into MP4 segments, restarted when it stops (no ROS)."""

# The stream is copied as the camera encodes it (-c copy): no decoding, no encoding, next to no
# CPU, the camera's own quality. Segments are fragmented MP4 (+frag_keyframe+empty_moov), each
# playable up to its last fragment even when the power goes. The password comes from the
# environment and is masked in everything kept or logged (ffmpeg prints the URL it failed on).

from __future__ import annotations

import os
import signal
import subprocess
import threading
from urllib.parse import quote

MASK = '***'
STEADY_SEC = 60.0  # a stream running this long restarts quickly again when it stops
MP4_OPTIONS = 'movflags=+frag_keyframe+empty_moov+default_base_moof'


def stream_url(host: str, port: int, user: str, password: str, channel: int) -> str:
    """Return the RTSP URL of a HIKMICRO channel (101 visible main, 201 thermal main)."""
    cred = f'{quote(user, safe="")}:{quote(password, safe="")}@' if user else ''
    return f'rtsp://{cred}{host}:{port}/Streaming/Channels/{channel}'


def masked(text: str, secret: str) -> str:
    """Return text with the secret (also URL-encoded) replaced by ***."""
    if not secret:
        return text
    return text.replace(quote(secret, safe=''), MASK).replace(secret, MASK)


def ffmpeg_command(ffmpeg: str, url: str, pattern: str, segment_sec: float, transport: str):
    """Return the command copying url's video into segments named by pattern (strftime)."""
    rtsp = ['-rtsp_transport', transport] if url.startswith('rtsp://') else []
    return [
        ffmpeg, '-nostdin', '-hide_banner', '-loglevel', 'warning', *rtsp, '-i', url,
        '-map', '0:v', '-c', 'copy', '-f', 'segment', '-segment_time', f'{segment_sec:g}',
        '-reset_timestamps', '1', '-strftime', '1', '-segment_format', 'mp4',
        '-segment_format_options', MP4_OPTIONS, pattern,
    ]  # fmt: skip


class StreamRecorder:
    """One stream of a session: ffmpeg while recording, restarted with a growing delay."""

    def __init__(self, name: str, url: str, secret: str, directory: str, options: dict, log):
        self.name, self.url, self.secret, self.directory = name, url, secret, directory
        self.options = options  # ffmpeg, segment_sec, transport, restart_sec
        self.log = log  # log(level, message), from any thread
        self.process: subprocess.Popen | None = None
        self.delay = options['restart_sec']
        self.retry_at = self.started_at = 0.0
        self.restarts = 0
        self.state, self.interrupted = 'off', False

    def start(self, now: float, popen=subprocess.Popen):
        """Start ffmpeg (again)."""
        os.makedirs(self.directory, exist_ok=True)
        pattern = os.path.join(self.directory, f'{self.name}_%Y%m%d_%H%M%S.mp4')
        o = self.options
        command = ffmpeg_command(o['ffmpeg'], self.url, pattern, o['segment_sec'], o['transport'])
        try:
            self.process = popen(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, start_new_session=True,
            )  # fmt: skip
        except OSError as error:
            self._failed(now, f'cannot start {o["ffmpeg"]}: {error}')
            return
        self.state, self.started_at, self.interrupted = 'recording', now, False
        threading.Thread(target=self._read, args=(self.process,), daemon=True).start()

    def poll(self, now: float, popen=subprocess.Popen):
        """Restart a stopped ffmpeg once its delay passed (call it every second or so)."""
        if self.state == 'recording' and self.process.poll() is not None:
            self._failed(now, f'ffmpeg ended ({self.process.returncode})')
        elif self.state == 'recording' and now - self.started_at >= STEADY_SEC:
            self.delay = self.options['restart_sec']
        if self.state == 'restarting' and now >= self.retry_at:
            self.restarts += 1
            self.start(now, popen)

    def running(self) -> bool:
        """Return True while ffmpeg runs."""
        return self.process is not None and self.process.poll() is None

    def interrupt(self):
        """Ask ffmpeg to close its segment and end (SIGINT, once); stop() waits for it."""
        process = self.process
        if process is not None and process.poll() is None and not self.interrupted:
            process.send_signal(signal.SIGINT)
            self.interrupted = True

    def stop(self, timeout: float = 5.0):
        """Let ffmpeg close its segment (SIGINT), kill it if it does not."""
        self.interrupt()
        process, self.process, self.state = self.process, None, 'off'
        if process is None:
            return
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout)

    def _failed(self, now: float, why: str):
        self.log('warning', f'{self.name}: {why}, again in {self.delay:.0f} s')
        self.state, self.retry_at = 'restarting', now + self.delay
        self.delay = min(60.0, self.delay * 2.0)

    def _read(self, process: subprocess.Popen):
        """Keep ffmpeg's messages, masked, in <name>.log (and the log); never block it."""
        path = os.path.join(self.directory, f'{self.name}.log')
        with open(path, 'a', encoding='utf-8') as out:
            for raw in iter(process.stderr.readline, b''):
                line = masked(raw.decode('utf-8', 'replace').rstrip(), self.secret)
                out.write(line + '\n')
                out.flush()
                self.log('debug', f'{self.name}: {line}')
