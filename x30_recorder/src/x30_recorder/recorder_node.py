"""recorder_node: the inspection cameras recorded on the perception host, on patrol or asked."""

# Everything is written on this host's disk as it comes, so nothing waits on the network and
# nothing is lost when a link drops: the raw thermal frames (.npz, degC per pixel, with the
# robot's pose and the camera's pan, tilt and zoom), the camera streams (MP4 segments copied as
# they are encoded) and the acoustic camera's data. A session runs while the path follower runs
# a route (patrol) or between recorder/start and recorder/stop (manual).

from __future__ import annotations

import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from x30_msgs.msg import NavigationStatus, RecorderStatus, SiteStatus
from x30_recorder.core import retention
from x30_recorder.core.config import RecorderConfig
from x30_recorder.core.patrol import Patrol
from x30_recorder.core.session import Session, stamp_name
from x30_recorder.core.writer import Writer
from x30_recorder.utils import convert as conv
from x30_recorder.utils.acoustic import AcousticInput
from x30_recorder.utils.streams import Streams
from x30_utils.io import write_thermal
from x30_utils.ros import LATCHED, RECORDED, SENSOR, X30Node, run

NAME = 'recorder_node'
POSE_MAX_AGE = 2.0  # s between a frame and the pose it is given
UNKNOWN = (math.nan, math.nan, math.nan)


class RecorderNode(X30Node):
    """Record the inspection cameras into sessions on this host's disk."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = cfg = self.declare_config(RecorderConfig).value
        self.root = os.path.expanduser(cfg.recordings_dir)
        os.makedirs(self.root, exist_ok=True)
        self.patrol = Patrol(cfg.patrol.stop_after_sec, cfg.patrol.silence_sec)
        self.writer = Writer(cfg.queue_size, self._write_failed)
        self.streams = Streams(cfg.video, self._log)
        self.session: Session | None = None
        self.manual = False
        self.pose = None  # (x, y, yaw), stamp
        self.ptz, self.ptz_moving = UNKNOWN, False
        self.site = ''
        self.last_frame = -math.inf
        self.free = math.inf
        self.problem = ''
        self.next_check = 0.0
        self.dropped_before = 0  # what the writer dropped before the session
        self._status_pub = self.latched(RecorderStatus, 'recorder/status')
        self.acoustic = AcousticInput(self, cfg.acoustic, self.writer)
        if cfg.thermal.enabled:
            from hikmicro_msgs.msg import PtzState

            self.create_subscription(Image, cfg.thermal.topic, self.on_thermal, RECORDED)
            self.create_subscription(PtzState, cfg.thermal.ptz_topic, self.on_ptz, SENSOR)
        self.create_subscription(PoseWithCovarianceStamped, cfg.pose_topic, self.on_pose, SENSOR)
        self.create_subscription(SiteStatus, cfg.site_topic, self.on_site, LATCHED)
        if cfg.patrol.enabled:
            status = cfg.patrol.status_topic
            self.create_subscription(NavigationStatus, status, self.on_navigation, LATCHED)
        self.create_service(Trigger, 'recorder/start', self.on_start)
        self.create_service(Trigger, 'recorder/stop', self.on_stop)
        self.add_shutdown_hook(self.close)
        self.every(1.0, self.tick, steady=True)
        for problem in self.streams.problems:
            self.log.error(problem)
        self.log.info('recorder ready', recordings=self.root, patrol=cfg.patrol.enabled,
                      streams=[name for name, _ in self.streams.specs])  # fmt: skip

    # ------------------------------------------------------------------ inputs

    def on_thermal(self, msg: Image):
        """Write a raw thermal frame while recording (at most one per min_interval_sec)."""
        session = self.session
        stamp = conv.stamp_of(msg, self.now())
        if session is None or stamp - self.last_frame < self.config.thermal.min_interval_sec:
            return
        try:
            temperature = conv.temperatures(msg)
        except ValueError as error:
            self.log.warning('thermal frame refused', reason=str(error), throttle=60.0)
            return
        self.last_frame = stamp
        pose = UNKNOWN
        if self.pose is not None and abs(self.pose[1] - stamp) <= POSE_MAX_AGE:
            pose = self.pose[0]
        ptz = self.ptz
        meta = {'frame_id': msg.header.frame_id, 'ptz_moving': self.ptz_moving, 'site': self.site}
        path = session.file('thermal', stamp_name(stamp) + '.npz')
        job = lambda: write_thermal(path, temperature, stamp, pose, ptz, meta)  # noqa: E731
        if self.writer.put('a thermal frame', job):
            session.count('thermal_frames')

    def on_ptz(self, msg):
        """Keep the camera's pan, tilt and zoom for the next frames."""
        self.ptz, self.ptz_moving = conv.ptz(msg), bool(msg.moving)

    def on_pose(self, msg: PoseWithCovarianceStamped):
        """Keep the robot's pose (map frame) for the next frames."""
        self.pose = conv.planar_pose(msg), conv.stamp_of(msg, self.now())

    def on_site(self, msg: SiteStatus):
        """Keep the active site for the sessions."""
        self.site = msg.site

    def on_navigation(self, msg: NavigationStatus):
        """Follow the path follower: a route running is a patrol."""
        self.patrol.update(int(msg.state), time.monotonic())

    def on_start(self, _request, response):
        """Service: record until recorder/stop (a patrol session goes on as a manual one)."""
        self.manual = True
        if self.session is None:
            self.begin('manual')
        response.success = self.session is not None
        response.message = self.session.path if self.session else self.problem
        return response

    def on_stop(self, _request, response):
        """Service: stop recording (a patrol still running starts a new session)."""
        self.manual = False
        stopped = self.session.path if self.session else ''
        self.end()
        response.success, response.message = True, stopped or 'not recording'
        return response

    # ------------------------------------------------------------------ sessions

    def begin(self, reason: str):
        """Start a session: its folder, the streams, the disk checked first."""
        now = self.now()
        self.check_disk(now)
        if self.free < self.config.disk.stop_free_gb:
            return
        info = {'site': self.site, 'host': os.uname().nodename,
                'streams': [name for name, _ in self.streams.specs]}  # fmt: skip
        try:
            self.session = Session(self.root, reason, now, info)
        except OSError as error:
            self.problem = f'no session: {error}'
            self.log.error(self.problem)
            return
        self.last_frame = -math.inf
        self.dropped_before = self.writer.dropped
        self.streams.start(self.session.file('video', '').rstrip('/'), time.monotonic())
        self.log.info('recording', session=self.session.path, reason=reason)

    def end(self):
        """End the session: streams closed, chunks written, its record final."""
        session = self.session
        if session is None:
            return
        self.acoustic.flush(self.now(), force=True)
        self.session = None
        self.streams.stop()
        session.count('dropped', self.writer.dropped - self.dropped_before)
        session.count('video_files', len(_files(os.path.join(session.path, 'video'), '.mp4')))
        stopped = self.now()
        self.writer.put('the session record', lambda: session.stop(stopped))
        self.log.info('recording stopped', session=session.path, **session.counts)

    def tick(self):
        """Every second: patrol or manual, streams restarted, chunks written, disk, status."""
        now, steady = self.now(), time.monotonic()
        wanted = self.manual or (self.config.patrol.enabled and self.patrol.wanted(steady))
        if wanted and self.session is None:
            self.begin('manual' if self.manual else 'patrol')
        elif not wanted and self.session is not None:
            self.end()
        if self.session is not None:
            self.streams.poll(steady)
            self.acoustic.flush(now)
            self.check_disk(now)
            if self.free < self.config.disk.stop_free_gb:
                self.log.error('disk full: recording stopped', free_gb=round(self.free, 1))
                self.end()
        self.publish_status(now)

    def check_disk(self, now: float, force: bool = False):
        """Make room (the oldest sessions go first) and measure what is free."""
        if not force and now < self.next_check:
            return
        disk = self.config.disk
        self.next_check = now + disk.check_sec
        active = self.session.path if self.session else ''
        removed, error = retention.prune(self.root, active, now, disk.keep_days, disk.min_free_gb)
        for path in removed:
            self.log.info('old recording removed (disk)', session=path)
        self.free = retention.free_gb(self.root)
        self.problem = error or (
            f'{self.free:.1f} GB free: not recording below {disk.stop_free_gb:g} GB'
            if self.free < disk.stop_free_gb
            else ''
        )

    def publish_status(self, now: float):
        """Publish recorder/status."""
        s = self.session
        msg = RecorderStatus(recording=s is not None, streams=self.streams.states())
        msg.free_gb = self.free if math.isfinite(self.free) else 0.0
        msg.dropped = self.writer.dropped - (self.dropped_before if s else 0)
        if s is not None:
            msg.reason, msg.session, msg.duration_sec = s.reason, s.path, s.duration(now)
            msg.thermal_frames = s.counts['thermal_frames']
            msg.acoustic_chunks = s.counts['acoustic_chunks']
            msg.video_files = len(_files(os.path.join(s.path, 'video'), '.mp4'))
        msg.message = '; '.join(p for p in (self.problem, *self.streams.problems) if p)
        msg.header.stamp = self.stamp()
        self._status_pub.publish(msg)

    def close(self):
        """Stop recording and write what is queued (shutdown)."""
        self.manual = False
        self.end()
        self.writer.close()

    def _write_failed(self, what: str, error: Exception):
        self.log.error('not written', what=what, error=str(error), throttle=10.0)

    def _log(self, level: str, message: str):
        getattr(self.log, level)(message, throttle=10.0 if level != 'debug' else 0.0)


def _files(directory: str, suffix: str) -> list[str]:
    try:
        return [n for n in os.listdir(directory) if n.endswith(suffix)]
    except OSError:
        return []


def main():
    """Run recorder_node."""
    run(RecorderNode, NAME)


if __name__ == '__main__':
    main()
