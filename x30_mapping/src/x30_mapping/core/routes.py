"""Route teaching: the localized pose sampled into a route CSV, rewritten atomically."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import os

from x30_mapping.core.config import RecorderConfig
from x30_utils.io import RoutePoint, SiteLayout, expand, valid_name, write_route
from x30_utils.math import Pose2D

ROUTE_TIME = '%Y-%m-%d_%H-%M-%S'


def route_path(directory: str, when: datetime) -> str:
    """Return a free <date>_<time>[-n].csv path in directory."""
    stem = when.strftime(ROUTE_TIME)
    for n in range(1000):
        name = f'{stem}.csv' if n == 0 else f'{stem}-{n}.csv'
        path = os.path.join(directory, name)
        if not os.path.exists(path):
            return path
    raise OSError(f'no free route name in {directory}')


@dataclass
class Recording:
    """One route being recorded into path."""

    path: str
    site: str
    points: list[RoutePoint] = field(default_factory=list)
    written: int = 0  # points on disk
    last_flush: float = 0.0

    @property
    def stops(self) -> int:
        """Return the number of stop points."""
        return sum(p.is_stop for p in self.points)


class RouteRecorder:
    """Recording state of route_recorder_node (pure: poses and time are passed in)."""

    def __init__(self, config: RecorderConfig):
        self.config = config
        self.recording: Recording | None = None

    @property
    def active(self) -> bool:
        """True while recording."""
        return self.recording is not None

    def directory(self, site: str) -> str:
        """Return where a route recorded now goes ('' when nowhere)."""
        if site and valid_name(site):
            return SiteLayout(expand(self.config.sites_dir), site).routes
        return expand(self.config.output_path)

    def start(self, site: str, localized: bool, when: datetime, now: float) -> tuple[bool, str]:
        """Start a recording; return (ok, message)."""
        if self.recording is not None:
            return False, f'already recording {self.recording.path}'
        if not localized:
            return False, 'not localized: localize on the site first'
        directory = self.directory(site)
        if not directory:
            return False, 'no active site (and no output_path): load a site first'
        try:
            os.makedirs(directory, exist_ok=True)
            path = route_path(directory, when)
        except OSError as error:
            return False, f'cannot record into {directory}: {error}'
        self.recording = Recording(path, site, last_flush=now)
        return True, f'recording {path}'

    def sample(self, pose: Pose2D | None, localized: bool) -> bool:
        """Add the pose when localized and far enough from the last waypoint."""
        rec = self.recording
        if rec is None or pose is None or not localized:
            return False
        if rec.points:
            last = rec.points[-1]
            if Pose2D(last.x, last.y).distance(pose) < self.config.min_spacing_m:
                return False
        rec.points.append(RoutePoint(pose.x, pose.y, pose.yaw, 0.0))
        return True

    def mark_stop(self, pose: Pose2D | None, localized: bool, dwell_sec: float, now: float):
        """Append a stop point and write the file; return (ok, message)."""
        rec = self.recording
        if rec is None:
            return False, 'not recording: call route/start first'
        if not localized or pose is None:
            return False, 'not localized: no pose for the stop point'
        if not dwell_sec > 0.0:
            return False, 'dwell_sec must be > 0 (a stop point waits there)'
        rec.points.append(RoutePoint(pose.x, pose.y, pose.yaw, float(dwell_sec)))
        self.flush(now, force=True)
        return True, f'stop point {rec.stops} at ({pose.x:.2f}, {pose.y:.2f}), {dwell_sec:.1f} s'

    def flush(self, now: float, force: bool = False) -> bool:
        """Write the route when it changed (at most every flush_sec); raise OSError."""
        rec = self.recording
        if rec is None or rec.written == len(rec.points):
            return False
        if not force and now - rec.last_flush < self.config.flush_sec:
            return False
        directory = os.path.dirname(rec.path)
        if not os.path.isdir(directory):
            raise FileNotFoundError(f'{directory} disappeared (site archived?)')
        write_route(rec.path, rec.points)
        rec.written, rec.last_flush = len(rec.points), now
        return True

    def stop(self, now: float) -> tuple[bool, str, str]:
        """End the recording and write it; return (ok, message, path)."""
        rec = self.recording
        if rec is None:
            return False, 'not recording', ''
        try:
            if not rec.points:
                return False, 'nothing recorded (not localized, or the robot did not move)', ''
            self.flush(now, force=True)
        except OSError as error:
            return False, f'route not saved: {error}', ''
        finally:
            self.recording = None
        return True, f'{len(rec.points)} waypoints, {rec.stops} stop points: {rec.path}', rec.path

    def on_site(self, site: str, now: float) -> str:
        """End a recording whose site is no longer active; return what happened ('' if none)."""
        rec = self.recording
        if rec is None or not rec.site or site == rec.site:
            return ''
        ok, message, _ = self.stop(now)
        state = 'saved' if ok else 'ended'
        return f'site changed to {site or "none"}: route of {rec.site} {state} ({message})'
