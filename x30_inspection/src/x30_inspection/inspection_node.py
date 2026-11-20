"""inspection_node: thermal and partial discharge inspection of the equipment at stop points."""

# The path follower calls inspection/inspect (its hooks/inspection) at every stop point; the node
# finds the site's inspection point nearest to where the robot stands, inspects its targets with
# the HIKMICRO PTZ camera (overheat analysis) and the Hertzino HZ-FA-371 (partial discharge),
# writes the report and answers. A stop with no point nearby answers at once, so it never holds
# the patrol. inspection/add_target teaches a target from the camera's current view.

from __future__ import annotations

from datetime import date, datetime
import threading
import time

from std_srvs.srv import Trigger

from x30_inspection.core import plan as plans
from x30_inspection.core import report, retention
from x30_inspection.core.config import InspectionConfig
from x30_inspection.core.discharge import AcousticCheck
from x30_inspection.core.runner import Runner
from x30_inspection.utils.acoustic import HertzinoAcoustic
from x30_inspection.utils.camera import HikmicroCamera
from x30_inspection.utils.convert import report_msg
from x30_inspection.utils.teach import add_target_from_camera
from x30_msgs.msg import (
    InspectionFinding,
    InspectionReport,
    InspectionStatus,
    SiteStatus,
    StopPointEvent,
)
from x30_msgs.srv import AddTarget
from x30_utils.ros import EVENTS, LATCHED, X30Node, run

NAME = 'inspection_node'
STATUS_PERIOD_SEC = 2.0
SHUTDOWN_WAIT_SEC = 3.0  # a running inspection cancels its move or detection meanwhile


class InspectionNode(X30Node):
    """Inspects the site's inspection points when the follower stops there."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(InspectionConfig)
        self.robot = self.declare_robot().value
        self.busy = threading.Lock()  # one inspection at a time
        self.closing = False
        self.site = ''
        self.stop = None  # (x, y, monotonic) of the stop the robot is at
        self.current, self.last = '', ('', 0)
        cfg = self.config.value
        self.camera = HikmicroCamera(self, cfg.camera, cfg.thermal.max_age_sec)
        self.acoustic = HertzinoAcoustic(self, cfg.acoustic)
        self.report_pub = self.create_publisher(InspectionReport, 'inspection/report', EVENTS)
        self.status_pub = self.latched(InspectionStatus, 'inspection/status')
        self.create_subscription(SiteStatus, 'site/status', self.on_site, LATCHED)
        self.create_subscription(StopPointEvent, 'navigation/stop_point', self.on_stop, EVENTS)
        # both wait for the cameras: beside the callbacks, which keep the status and the stop
        self.create_service(Trigger, 'inspection/inspect', self.srv_inspect, concurrent=True)
        self.create_service(AddTarget, 'inspection/add_target', self.srv_add_target,
                            concurrent=True)  # fmt: skip
        self.every(STATUS_PERIOD_SEC, self.publish_status)
        self.add_shutdown_hook(self.on_shutdown)
        self.log.info('inspection', enabled=cfg.enabled, camera=cfg.camera.namespace,
                      acoustic=cfg.acoustic.namespace, sites_dir=cfg.sites_dir)  # fmt: skip
        self.prune_reports()
        self.publish_status()

    # ---------------------------------------------------------------- inputs
    def on_site(self, msg: SiteStatus):
        """Follow the active site."""
        self.site = msg.site

    def on_stop(self, msg: StopPointEvent):
        """Remember where the robot stops; forget it when it leaves."""
        if msg.event == StopPointEvent.ARRIVED:
            self.stop = (msg.x, msg.y, time.monotonic())
        elif msg.event in (StopPointEvent.DEPARTED, StopPointEvent.SKIPPED):
            self.stop = None

    def robot_xy(self) -> tuple[float, float] | None:
        """Return where the robot stands: the stop it is at, else map -> base_link."""
        stop = self.stop
        if stop is not None and time.monotonic() - stop[2] < self.config.stop_max_age_sec:
            return stop[0], stop[1]
        result = self.tf.lookup(self.robot.frames.map, self.robot.frames.base)
        if result is None:
            return None
        pose = result.pose2d
        return pose.x, pose.y

    def plan(self) -> plans.Plan:
        """Return the plan of the active site (empty without a site)."""
        if not self.site:
            return plans.Plan()
        return plans.load(plans.plan_path(self.config.sites_dir, self.site))

    # ---------------------------------------------------------------- inspect
    def srv_inspect(self, _request, response):
        """Service (follower hook): inspect the point where the robot stands."""
        response.success, response.message = self.inspect()
        self.log.info('inspect', ok=response.success, detail=response.message)
        return response

    def inspect(self) -> tuple[bool, str]:
        """Run one inspection point; (ok, message)."""
        if not self.config.enabled:
            return True, 'inspection off'
        if self.closing:
            return False, 'shutting down'
        if not self.busy.acquire(blocking=False):
            return False, 'an inspection is running'
        try:
            return self.inspect_here()
        except (plans.PlanError, OSError) as error:
            self.log.error('inspection', error=str(error))
            return False, str(error)
        finally:
            self.current = ''
            self.busy.release()
            self.publish_status()

    def inspect_here(self) -> tuple[bool, str]:
        """Find the point here and inspect its targets."""
        here = self.robot_xy()
        if here is None:
            return False, 'robot pose unknown (not localized?)'
        point = self.plan().nearest(here[0], here[1], self.config.match_radius_m)
        if point is None:
            return True, f'no inspection point within {self.config.match_radius_m:.1f} m'
        self.current = point.name
        self.publish_status()
        when = datetime.now()
        directory = report.report_dir(self.config.reports_dir, when, point.name)
        cfg = self.config.value
        runner = Runner(cfg, self.camera)
        runner.checks.append(AcousticCheck(self.acoustic, cfg.acoustic, self.is_closing))
        result = runner.inspect(point, lambda t: report.evidence_prefix(directory, t.name),
                                self.is_closing)  # fmt: skip
        report.write(directory, report.to_dict(result, self.site, here[0], here[1], when))
        self.prune_reports()
        msg = report_msg(result, self.site, here[0], here[1], directory, self.stamp())
        self.report_pub.publish(msg)
        self.last = (point.name, msg.severity)
        if msg.severity >= InspectionFinding.WARNING:
            self.log.warning('finding', point=point.name, detail=result.message,
                             severity=report.severity_name(msg.severity))  # fmt: skip
        return result.state != 'failed', result.message

    def prune_reports(self):
        """Keep the reports within the disk: old days go, the oldest first (never today's)."""
        cfg = self.config.value
        removed, error = retention.prune(
            cfg.reports_dir, date.today(), cfg.reports_keep_days, cfg.reports_min_free_gb
        )
        if removed:
            self.log.warning('old reports removed', days=','.join(removed),
                             keep_days=cfg.reports_keep_days,
                             min_free_gb=cfg.reports_min_free_gb)  # fmt: skip
        if error:
            self.log.error('reports not pruned', error=error)

    # ---------------------------------------------------------------- teach
    def srv_add_target(self, request, response):
        """Service: teach a target from the camera's pan, tilt and zoom where the robot stands."""
        ok, message, point, count = add_target_from_camera(self, request)
        response.success, response.message, response.point, response.targets = (
            ok, message, point, count)  # fmt: skip
        self.log.info('add_target', ok=ok, detail=message)
        self.publish_status()
        return response

    # ---------------------------------------------------------------- status
    def publish_status(self):
        """Publish inspection/status (latched)."""
        if self.closing:
            return
        msg = InspectionStatus(site=self.site, current=self.current)
        msg.header.stamp = self.stamp()
        if not self.config.enabled:
            msg.state = InspectionStatus.DISABLED
        elif self.current:
            msg.state = InspectionStatus.INSPECTING
        try:
            plan = self.plan()
            msg.points, msg.targets = len(plan.points), plan.targets
        except plans.PlanError as error:
            msg.message = f'plan: {error}'
        msg.thermal_connected = self.camera.connected()
        msg.acoustic_connected = self.acoustic.connected()
        msg.last_point, msg.last_severity = self.last
        msg.ambient_c = float(self.config.thermal.ambient_c)
        self.status_pub.publish(msg)

    def is_closing(self) -> bool:
        """Return True once the node shuts down (a running inspection stops)."""
        return self.closing

    def on_shutdown(self):
        """Stop a running inspection (its move or detection is cancelled), then the clients."""
        self.closing = True
        if self.busy.acquire(timeout=SHUTDOWN_WAIT_SEC):
            self.busy.release()
        else:
            self.log.warning('inspection still running at shutdown', timeout=SHUTDOWN_WAIT_SEC)
        self.camera.close()
        self.acoustic.close()


def main():
    """Run inspection_node."""
    run(InspectionNode, NAME)


if __name__ == '__main__':
    main()
