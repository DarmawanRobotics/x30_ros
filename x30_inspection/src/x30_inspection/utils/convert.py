"""Core results -> x30_msgs inspection messages."""

from __future__ import annotations

import math

from x30_inspection.core.runner import PointResult
from x30_inspection.core.thermal import worst
from x30_msgs.msg import InspectionFinding, InspectionReport

STATES = {
    'done': InspectionReport.DONE,
    'partial': InspectionReport.PARTIAL,
    'failed': InspectionReport.FAILED,
    'no_targets': InspectionReport.NO_TARGETS,
}


def finite(value: float) -> float:
    """Return value, 0.0 for NaN or infinity (messages must stay readable)."""
    return float(value) if math.isfinite(value) else 0.0


def report_msg(result: PointResult, site: str, x: float, y: float, directory: str, stamp):
    """Return the InspectionReport of a point."""
    msg = InspectionReport(site=site, point=result.point, x=float(x), y=float(y))
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.state = STATES[result.state]
    msg.severity = int(worst(result.findings))
    msg.directory = directory
    msg.duration_sec = float(result.duration_sec)
    msg.message = result.message
    for target in result.targets:
        for f in target.findings:
            msg.findings.append(
                InspectionFinding(
                    target=f.target,
                    sensor=f.sensor,
                    criterion=f.criterion,
                    label=f.label,
                    severity=int(f.severity),
                    value=finite(f.value),
                    reference=finite(f.reference),
                    delta=finite(f.delta),
                    unit=f.unit,
                    message=f.message,
                    files=list(target.files),
                )
            )
    return msg
