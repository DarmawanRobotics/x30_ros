"""The robot as NETRA sees it (robot API v1 telemetry), kept up to date from the stack's topics."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import BatteryState

from x30_bridge.core import status
from x30_fleet.core import health
from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    InspectionStatus,
    LocalizationStatus,
    NavigationStatus,
    RecorderStatus,
    SiteStatus,
    StopPointEvent,
)
from x30_utils.ros import EVENTS, LATCHED, SENSOR

STOP_EVENTS = {0: 'arrived', 1: 'inspected', 2: 'departed', 3: 'skipped'}  # StopPointEvent
RECORDER = ('recording', 'reason', 'session', 'duration_sec', 'thermal_frames', 'acoustic_chunks',
            'video_files', 'dropped', 'streams', 'free_gb', 'message')  # fmt: skip
INSPECTION = ('state', 'site', 'points', 'targets', 'thermal_connected', 'acoustic_connected',
              'current', 'last_point', 'last_severity', 'ambient_c', 'message')  # fmt: skip


def finite(value, digits: int) -> float | None:
    """Return value rounded for JSON, None when it is not a finite number."""
    return (
        round(value, digits) if isinstance(value, (int, float)) and math.isfinite(value) else None
    )


class FleetTelemetry:
    """Status subscriptions; events for NETRA go to node.send(message, durable)."""

    def __init__(self, node, config):
        self.node = node
        self.state = {
            'mode': 'auto',
            'nav': {'state': 'NAV_INACTIVE', 'message': ''},
            'mission': '',
            'localization': 'none',
            'battery': None,
            'driver': None,
            'people': 0,
            'inspection': None,  # Pashupati's VLM inspection, none on the X30
            'people_detection': None,
            'health': None,
            'estop': False,
            'field': None,
            'recorder': None,  # the X30's: x30_recorder on the perception host
            'substation': None,  # the X30's: x30_inspection
        }
        self.site = ''
        self.found: dict = {}
        sub = node.create_subscription
        sub(ControlStatus, 'control/status', self.on_control, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, EVENTS)
        sub(StopPointEvent, 'navigation/stop_point', self.on_stop, EVENTS)
        sub(LocalizationStatus, 'localization/status', self.on_localization, LATCHED)
        sub(SiteStatus, 'site/status', self.on_site, LATCHED)
        sub(HealthStatus, 'health/status', self.on_health, LATCHED)
        sub(DiagnosticArray, config.health_diagnostics_topic, self.on_findings, SENSOR)
        sub(BatteryState, config.battery_topic, self.on_battery, SENSOR)
        sub(DiagnosticArray, config.driver_diagnostics_topic, self.on_driver, SENSOR)
        sub(RecorderStatus, 'recorder/status', self.on_recorder, LATCHED)
        sub(InspectionStatus, 'inspection/status', self.on_inspection, LATCHED)

    def on_control(self, msg: ControlStatus):
        """Mode, field and the e-stop (an alert when it engages)."""
        engaged = bool(msg.estop)
        if engaged and not self.state['estop']:
            alert = self.node.patrol.estop(list(msg.estop_reasons), self.node.pose())
            self.node.send(alert, durable=True)
        self.state.update(mode=status.mode_name(msg.mode), estop=engaged,
                          field=status.field_name(msg.field))  # fmt: skip

    def on_nav(self, msg: NavigationStatus):
        """Follower state; alerts, laps and the end of a patrol."""
        name = status.nav_name(int(msg.state))
        self.state['nav'] = {'state': name, 'message': msg.message}
        events = self.node.patrol.navigation(
            name, msg.message, int(msg.index), int(msg.count), self.node.pose()
        )
        for event in events:
            self.node.send(event, durable=True)
        self.state['mission'] = self.node.patrol.mission()

    def on_stop(self, msg: StopPointEvent):
        """Forward a stop-point event."""
        event = self.node.patrol.stop_point(
            STOP_EVENTS.get(int(msg.event), 'skipped'), int(msg.index), msg.message,
            float(msg.x), float(msg.y), round(math.degrees(msg.yaw), 1),
        )  # fmt: skip
        self.node.send(event, durable=True)

    def on_localization(self, msg: LocalizationStatus):
        """'none' or the source of the last correction."""
        self.state['localization'] = status.localization_text(int(msg.state), msg.source)

    def on_site(self, msg: SiteStatus):
        """Keep the active site (NETRA's routes and plans go into it)."""
        self.site = msg.site

    def on_findings(self, msg: DiagnosticArray):
        """Keep x30_health's findings (the computer's figures, topic rates)."""
        self.found = health.findings(
            (s.name, {kv.key: kv.value for kv in s.values}) for s in msg.status
        )

    def on_health(self, msg: HealthStatus):
        """ok, level, issues, topic rates and the computer."""
        self.state['health'] = health.summary(int(msg.level), msg.issues, msg.speed_scale,
                                              self.found)  # fmt: skip

    def on_battery(self, msg: BatteryState):
        """Battery readings."""
        self.state['battery'] = {
            'pct': finite(msg.percentage * 100.0, 0),
            'v': finite(msg.voltage, 1),
            'a': finite(msg.current, 1),
            'temp': finite(msg.temperature, 0),
        }

    def on_driver(self, msg: DiagnosticArray):
        """Keep the driver's status line."""
        if msg.status:
            self.state['driver'] = {
                'level': int(msg.status[0].level),
                'msg': msg.status[0].message,
            }

    def on_recorder(self, msg: RecorderStatus):
        """Keep the camera recorder's state: recording, why, how much, the disk left."""
        values = {name: getattr(msg, name) for name in RECORDER}
        self.state['recorder'] = {**status.recorder_summary(values), 'at': time.time()}

    def on_inspection(self, msg: InspectionStatus):
        """Keep the substation inspection's state: the point now, the last one, its severity."""
        values = {name: getattr(msg, name) for name in INSPECTION}
        self.state['substation'] = {**status.inspection_summary(values), 'at': time.time()}
