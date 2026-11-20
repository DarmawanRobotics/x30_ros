"""What the handheld's `state` carries, kept up to date from the stack's status topics."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

from x30_bridge.core import protocol, status
from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    InspectionStatus,
    LocalizationStatus,
    NavigationStatus,
    RecorderStatus,
    TripStatus,
    WaypointPath,
)
from x30_utils.ros import LATCHED, SENSOR

ROUTE_POINTS = 600  # the route sent to the handheld, at most
STALE_SEC = {'recorder': 5.0, 'inspection': 10.0}  # published every 1 s and every 2 s
RECORDER_FIELDS = (
    'recording',
    'reason',
    'session',
    'duration_sec',
    'thermal_frames',
    'acoustic_chunks',
    'video_files',
    'dropped',
    'streams',
    'free_gb',
    'message',
)
INSPECTION_FIELDS = (
    'state',
    'site',
    'points',
    'targets',
    'thermal_connected',
    'acoustic_connected',
    'current',
    'last_point',
    'last_severity',
    'ambient_c',
    'message',
)


def fields(msg, names) -> dict:
    """Return the named fields of a message as a dict."""
    return {name: getattr(msg, name) for name in names}


class Telemetry:
    """Subscribes to the status topics and keeps the handheld's view of the robot."""

    def __init__(self, node, config, on_route):
        self.on_route = on_route  # called when the route changed
        self.state = {
            'mode': 'auto',
            'estop': False,
            'field': None,
            'nav': 'NAV_INACTIVE',
            'nav_msg': '',
            'mission': '',
            'localization': 'none',
            'recording': False,
            'battery': None,
            'driver': None,
            'cmd': [0.0, 0.0, 0.0],
            'health': None,
            'trip': None,
        }
        self.route: list = []
        self.findings: dict = {}
        self.parts = {'recorder': (None, 0.0), 'inspection': (None, 0.0)}
        sub = node.create_subscription
        sub(ControlStatus, 'control/status', self.on_control, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, LATCHED)
        sub(LocalizationStatus, 'localization/status', self.on_localization, LATCHED)
        sub(HealthStatus, 'health/status', self.on_health, LATCHED)
        sub(DiagnosticArray, config.health_diagnostics_topic, self.on_findings, SENSOR)
        sub(WaypointPath, 'navigation/waypoints', self.on_waypoints, LATCHED)
        sub(TripStatus, 'odometry/trip', self.on_trip, LATCHED)
        sub(BatteryState, config.battery_topic, self.on_battery, SENSOR)
        sub(DiagnosticArray, config.driver_diagnostics_topic, self.on_driver, SENSOR)
        sub(Twist, config.cmd_topic, self.on_cmd, SENSOR)
        sub(RecorderStatus, 'recorder/status', self.on_recorder, LATCHED)
        sub(InspectionStatus, 'inspection/status', self.on_inspection, LATCHED)

    def on_control(self, msg: ControlStatus):
        """Mode, e-stop and protective field of the control mux."""
        self.state.update(mode=status.mode_name(msg.mode), estop=bool(msg.estop),
                          field=status.field_name(msg.field))  # fmt: skip

    def on_nav(self, msg: NavigationStatus):
        """Keep the follower's state, its message and a line about the route."""
        name = status.nav_name(int(msg.state))
        line = status.mission_text(name, list(msg.pause_reasons), int(msg.index), int(msg.count))
        self.state.update(nav=name, nav_msg=msg.message, mission=line)

    def on_localization(self, msg: LocalizationStatus):
        """'none' or the source of the last correction."""
        self.state['localization'] = status.localization_text(int(msg.state), msg.source)

    def on_findings(self, msg: DiagnosticArray):
        """Keep x30_health's findings (the computer's figures)."""
        self.findings = {status.finding_name(s.name): {kv.key: kv.value for kv in s.values}
                         for s in msg.status}  # fmt: skip

    def on_health(self, msg: HealthStatus):
        """ok, issues and the computer's load."""
        self.state['health'] = status.health_summary(int(msg.level), msg.issues, self.findings)

    def on_waypoints(self, msg: WaypointPath):
        """Keep the loaded route and push it to the handheld."""
        points = [[round(w.x, 2), round(w.y, 2), round(w.dwell_sec, 1)] for w in msg.waypoints]
        self.route = protocol.simplify_route(points, ROUTE_POINTS)
        self.on_route()

    def on_trip(self, msg: TripStatus):
        """Distance and speeds since the last trip_reset."""
        since = msg.since.to_sec()
        self.state['trip'] = status.trip_summary(msg.distance, msg.moving_sec, msg.speed,
                                                 msg.average, msg.max_speed, since)  # fmt: skip

    def on_battery(self, msg: BatteryState):
        """Battery readings."""
        self.state['battery'] = {
            'pct': status.finite(msg.percentage * 100.0, 0),
            'v': status.finite(msg.voltage, 1),
            'a': status.finite(msg.current, 1),
            'temp': status.finite(msg.temperature, 0),
        }

    def on_driver(self, msg: DiagnosticArray):
        """Keep the driver's status line."""
        if msg.status:
            first = msg.status[0]
            self.state['driver'] = {'level': int(first.level), 'msg': first.message}

    def on_cmd(self, msg: Twist):
        """Keep the command the control mux sends to the robot."""
        self.state['cmd'] = [round(msg.linear.x, 2), round(msg.linear.y, 2),
                             round(msg.angular.z, 2)]  # fmt: skip

    def on_recorder(self, msg: RecorderStatus):
        """Keep the camera recorder's status (perception host)."""
        summary = status.recorder_summary(fields(msg, RECORDER_FIELDS))
        self.parts['recorder'] = (summary, time.monotonic())

    def on_inspection(self, msg: InspectionStatus):
        """Keep the substation inspection's status."""
        summary = status.inspection_summary(fields(msg, INSPECTION_FIELDS))
        self.parts['inspection'] = (summary, time.monotonic())

    def part(self, key: str):
        """Return the recorder's or the inspection's part of `state`; None when none ever ran."""
        summary, seen = self.parts[key]
        if summary is None:
            return None
        return {**summary, 'offline': time.monotonic() - seen > STALE_SEC[key]}
