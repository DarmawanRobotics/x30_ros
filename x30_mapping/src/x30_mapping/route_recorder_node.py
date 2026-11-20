"""route_recorder_node: teach a route by driving it, saved as a CSV route of the site."""

from __future__ import annotations

from datetime import datetime

from std_msgs.msg import String
from std_srvs.srv import Trigger

from x30_mapping.core.config import RecorderConfig
from x30_mapping.core.routes import RouteRecorder
from x30_msgs.msg import LocalizationStatus, SiteStatus
from x30_msgs.srv import MarkStopPoint
from x30_utils.math import Pose2D
from x30_utils.ros import EVENTS, LATCHED, X30Node, run

NAME = 'route_recorder_node'
TF_SETTLE_SEC = 2.0  # TF fills the buffer within this after the listener started (no warning)


class RouteRecorderNode(X30Node):
    """Record the localized robot path into the active site's routes."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        live = self.declare_config(RecorderConfig)
        self.frames = self.declare_robot().value.frames
        self.recorder = RouteRecorder(live.value)
        live.on_change(self._on_config)
        self.site = ''
        self.localized = False
        self._tf_since: float | None = None  # when the first lookup started the TF listener
        self._saved_pub = self.create_publisher(String, 'route/saved', EVENTS)
        self.create_subscription(SiteStatus, 'site/status', self._on_site, LATCHED)
        self.create_subscription(
            LocalizationStatus, 'localization/status', self._on_localization, LATCHED
        )
        self.create_service(Trigger, 'route/start', self._on_start)
        self.create_service(Trigger, 'route/stop', self._on_stop)
        self.create_service(MarkStopPoint, 'route/mark_stop_point', self._on_mark)
        self.every(1.0 / live.value.record_rate_hz, self._tick)
        self.add_shutdown_hook(self._save_on_shutdown)
        self.log.info('route recorder ready', sites_dir=live.value.sites_dir)

    def _on_config(self, old: RecorderConfig, new: RecorderConfig):
        self.recorder.config = new

    def _now(self) -> float:
        return self.now()

    def _pose(self) -> Pose2D | None:
        """Return the fresh map->base pose, or None (warned once TF had time to arrive)."""
        if self._tf_since is None:
            self._tf_since = self._now()
        result = self.tf.fresh(
            self.frames.map, self.frames.base, self.recorder.config.max_tf_age_sec
        )
        if result is None:
            if self._now() - self._tf_since > TF_SETTLE_SEC:
                self.log.warning('no fresh map->base TF', error=self.tf.error, throttle=5.0)
            return None
        return result.pose2d

    def _on_site(self, msg: SiteStatus):
        self.site = msg.site
        message = self.recorder.on_site(msg.site, self._now())
        if message:
            self.log.warning(message)

    def _on_localization(self, msg: LocalizationStatus):
        self.localized = msg.state == LocalizationStatus.LOCALIZED

    def _tick(self):
        """Sample the pose while recording and write the file every flush_sec."""
        if not self.recorder.active:
            return
        self.recorder.sample(self._pose() if self.localized else None, self.localized)
        try:
            self.recorder.flush(self._now())
        except OSError as error:
            _, message, _ = self.recorder.stop(self._now())
            self.log.error(
                'recording ended: route file not writable', error=str(error), result=message
            )

    def _on_start(self, request, response):
        """Service: start recording a route into the active site."""
        if self.localized:
            self._pose()  # start listening to TF now
        ok, message = self.recorder.start(self.site, self.localized, datetime.now(), self._now())
        response.success, response.message = ok, message
        (self.log.info if ok else self.log.warning)(f'route/start: {message}')
        return response

    def _on_stop(self, request, response):
        """Service: stop recording and save the route (route/saved announces it)."""
        ok, message, path = self.recorder.stop(self._now())
        if ok:
            self._saved_pub.publish(String(data=path))
        response.success, response.message = ok, message
        (self.log.info if ok else self.log.warning)(f'route/stop: {message}')
        return response

    def _on_mark(self, request, response):
        """Service: the current pose becomes a stop point waiting dwell_sec."""
        pose = self._pose() if self.localized else None
        try:
            ok, message = self.recorder.mark_stop(
                pose, self.localized, float(request.dwell_sec), self._now()
            )
        except OSError as error:
            ok, message = False, f'stop point kept in memory, file not written: {error}'
        response.success, response.message = ok, message
        (self.log.info if ok else self.log.warning)(f'route/mark_stop_point: {message}')
        return response

    def _save_on_shutdown(self):
        if self.recorder.active:
            ok, message, _ = self.recorder.stop(self._now())
            self.log.warning('shutdown while recording', saved=ok, result=message)


def main():
    """Run route_recorder_node."""
    run(RouteRecorderNode, NAME)


if __name__ == '__main__':
    main()
