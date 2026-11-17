"""Status messages -> core inputs, the report -> HealthStatus and DiagnosticArray."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray

from x30_health.core.checks.state import ControlInput, LocalizationInput, OdometryInput
from x30_health.core.report import Report
from x30_msgs.msg import ControlStatus, HealthStatus, LocalizationStatus, OdometryStatus
from x30_utils.ros.diagnostics import status


def odometry_input(msg: OdometryStatus) -> OdometryInput:
    """Return the core input of an OdometryStatus."""
    return OdometryInput(int(msg.state), msg.message)


def localization_input(msg: LocalizationStatus) -> LocalizationInput:
    """Return the core input of a LocalizationStatus."""
    return LocalizationInput(
        int(msg.state), float(msg.sigma_xy), float(msg.sigma_yaw), msg.message
    )


def control_input(msg: ControlStatus) -> ControlInput:
    """Return the core input of a ControlStatus."""
    return ControlInput(bool(msg.estop), tuple(msg.estop_reasons), int(msg.field))


def health_status(report: Report, stamp) -> HealthStatus:
    """Return the HealthStatus of a report."""
    msg = HealthStatus(
        level=int(report.level), issues=report.lines(), speed_scale=float(report.speed_scale)
    )
    msg.header.stamp = stamp
    return msg


def diagnostic_array(report: Report, prefix: str, stamp) -> DiagnosticArray:
    """Return one DiagnosticStatus per finding, named '<prefix>: <finding>'."""
    array = DiagnosticArray()
    array.header.stamp = stamp
    array.status = [
        status(f'{prefix}: {f.name}', f.level.name.lower(), f.message, **f.values)
        for f in report.findings
    ]
    return array
