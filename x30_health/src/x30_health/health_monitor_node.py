"""health_monitor_node: one typed health signal (health/status) and diagnostics for the robot."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray
import rosgraph
import rospy

from x30_health.core.checks import Check, Issue, Level, registry
from x30_health.core.checks.clock import ClockCheck
from x30_health.core.checks.nodes import NodesCheck
from x30_health.core.checks.scan import SafetyScanCheck
from x30_health.core.checks.sensors import SensorCheck
from x30_health.core.checks.state import StateCheck
from x30_health.core.checks.system import SystemCheck
from x30_health.core.checks.tf import TfCheck
from x30_health.core.config import HealthConfig
from x30_health.core.report import Report, aggregate
from x30_health.core.stamp import ros1_stamp, starts_with_header
from x30_health.utils.convert import (
    control_input,
    diagnostic_array,
    health_status,
    localization_input,
    odometry_input,
)
from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    OdometryStatus,
    SectorScan,
)
from x30_utils.ros import LATCHED, SENSOR, X30Node, run

NAME = 'health_monitor_node'


class HealthMonitorNode(X30Node):
    """health/status (HealthStatus, latched) and diagnostics from topics, states, TF, computer."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(HealthConfig)
        self.robot = self.declare_robot()
        c = self.config.value
        grace_until = time.monotonic() + c.startup_grace_sec
        self.sensor_check = SensorCheck(c.sensors, self._resolve)
        self.system_check = SystemCheck(c.system)
        self.node_check = NodesCheck(c.nodes, grace_until)
        self.state_check = StateCheck(c.state)
        self.scan_check = None
        if c.state.safety_scan_topic:
            self.scan_check = SafetyScanCheck(
                c.state.safety_scan_topic, c.state.scan_timeout_sec, grace_until
            )
        self.tf_check = TfCheck(c.tf, self._chain()) if c.tf.enabled else None
        self.clock_check = ClockCheck(c.clock) if c.clock.enabled else None
        self.extra_checks: list[Check] = [registry.create(name) for name in c.extra_checks]
        self.config.on_change(self.reconfigure)
        self.raw_subscriptions: dict[str, object] = {}
        self.has_header: dict[str, bool] = {}  # per topic, from its message definition
        self.last_report: Report | None = None

        self.status_pub = self.latched(HealthStatus, 'health/status')
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self._subscribe_states()
        self.every(c.sensors.discover_period_sec, self.discover, steady=True)
        self.every(c.period_sec, self.publish_report, steady=True)
        self.discover()
        self.log.info(
            'watching',
            topics=len(self.sensor_check.topics),
            nodes=len(c.nodes.expected),
            tf=c.tf.enabled,
            ntp=c.clock.enabled,
        )

    def _chain(self) -> tuple[str, str]:
        frames = self.robot.frames
        return frames.odom, frames.base

    def _resolve(self, topic: str) -> str:
        """Return the fully qualified name of a watched topic as this node sees it."""
        return rospy.resolve_name(topic)

    def _subscribe_states(self):
        c = self.config.state
        mono = time.monotonic
        handlers = (
            (OdometryStatus, c.odometry_topic, self.state_check.on_odometry, odometry_input),
            (
                LocalizationStatus,
                c.localization_topic,
                self.state_check.on_localization,
                localization_input,
            ),
            (ControlStatus, c.control_topic, self.state_check.on_control, control_input),
        )
        for msg_type, topic, handler, convert in handlers:
            if topic:
                self.create_subscription(
                    msg_type, topic, lambda m, h=handler, f=convert: h(mono(), f(m)), LATCHED
                )
        if self.scan_check is not None:
            self.create_subscription(
                SectorScan,
                c.safety_scan_topic,
                lambda m: self.scan_check.on_scan(mono(), m.ranges),
                SENSOR,
            )

    def reconfigure(self, _old: HealthConfig, new: HealthConfig):
        """Apply live parameter changes to the checks."""
        self.sensor_check.configure(new.sensors)
        self.system_check.configure(new.system)
        self.node_check.configure(new.nodes)
        self.state_check.configure(new.state)
        if self.scan_check is not None:
            self.scan_check.configure(new.state.scan_timeout_sec)
        if self.tf_check is not None:
            self.tf_check.configure(new.tf)
        if self.clock_check is not None:
            self.clock_check.configure(new.clock)

    def discover(self):
        """Subscribe (raw, any type, no deserialisation) to watched topics once published."""
        try:
            published = {topic for topic, _type in rospy.get_published_topics()}
        except (rospy.ROSException, OSError):  # the master is away: next time
            return
        for topic in self.sensor_check.topics:
            if topic in self.raw_subscriptions or topic not in published:
                continue
            self.raw_subscriptions[topic] = self.create_any_subscription(
                topic, lambda data, connection, t=topic: self._on_raw(t, data, connection), SENSOR
            )
            self.sensor_check.subscribed(topic)

    def _on_raw(self, topic: str, data: bytes, connection: dict):
        header = self.has_header.get(topic)
        if header is None:
            header = starts_with_header(connection.get('message_definition', ''))
            self.has_header[topic] = header
        age = None
        if header:
            stamp = ros1_stamp(data)
            if stamp is not None:
                age = self.clock_now() - stamp
        self.sensor_check.on_message(topic, time.monotonic(), age)

    def clock_now(self) -> float:
        """Return the node clock (ROS time, simulated or not) in seconds."""
        return self.now()

    def _observe(self):
        """Feed the inputs that are polled rather than subscribed."""
        if self.config.nodes.expected:
            self.node_check.update(running_nodes())
        if self.tf_check is not None:
            self.tf_check.chain = self._chain()
            chain_ok = self.tf.lookup(*self.tf_check.chain) is not None
            self.tf_check.observe(self.clock_now(), self.tf.edge_ages(), chain_ok)

    def _checks(self) -> list[Check]:
        checks = [self.sensor_check, self.system_check, self.node_check, self.state_check]
        optional = [self.scan_check, self.tf_check, self.clock_check]
        return checks + [c for c in optional if c is not None] + self.extra_checks

    def evaluate(self) -> Report:
        """Run every check; a failing check is reported, never silently skipped."""
        self._observe()
        now = time.monotonic()
        findings: list[Issue] = []
        for check in self._checks():
            try:
                findings += check.run(now)
            except Exception as error:
                name = type(check).__name__
                self.log.exception('check failed', check=name, throttle=10.0)
                findings.append(Issue(Level.WARN, f'check:{name}', f'{name} failed: {error}'))
        return aggregate(findings, self.config.degradation)

    def publish_report(self):
        """Publish health/status and diagnostics, log changes of the level or the issues."""
        report = self.evaluate()
        stamp = self.stamp()
        self.status_pub.publish(health_status(report, stamp))
        prefix = self.config.diagnostics_prefix
        self.diagnostics_pub.publish(diagnostic_array(report, prefix, stamp))
        self._log_changes(report)
        self.last_report = report

    def _log_changes(self, report: Report):
        last = self.last_report
        same = last is not None and last.level is report.level
        if same and issue_names(report) == issue_names(last):
            return
        fields = {'level': report.level.name, 'speed_scale': report.speed_scale}
        if report.level is Level.OK:
            self.log.info('health ok', **fields)
        elif report.level is Level.WARN:
            self.log.warning('health degraded', issues='; '.join(report.lines()), **fields)
        else:
            self.log.error('health error', issues='; '.join(report.lines()), **fields)


def running_nodes() -> list[tuple[str, str]]:
    """Return (name, namespace) of every node the master knows."""
    try:
        state = rosgraph.Master(rospy.get_name()).getSystemState()
    except (rosgraph.MasterException, OSError):  # the master is away: none
        return []
    nodes = {node for kind in state for _topic, names in kind for node in names}
    return [(n.rsplit('/', 1)[1], n.rsplit('/', 1)[0] or '/') for n in nodes]


def issue_names(report: Report) -> set[str]:
    """Return the names of the issues of a report."""
    return {i.name for i in report.issues}


def main():
    """Run the health monitor."""
    run(HealthMonitorNode, NAME)


if __name__ == '__main__':
    main()
