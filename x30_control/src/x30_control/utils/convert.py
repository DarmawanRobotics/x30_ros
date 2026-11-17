"""Messages <-> core types of the control mux."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
import numpy as np

from x30_control.core.arbiter import ArbiterState, Mode
from x30_control.core.command import Command
from x30_control.core.field import FieldState
from x30_control.core.scan import ScanData
from x30_msgs.msg import ControlStatus, SectorScan
from x30_utils.ros.time import to_sec

FIELD = {
    FieldState.OFF: ControlStatus.FIELD_OFF,
    FieldState.CLEAR: ControlStatus.FIELD_CLEAR,
    FieldState.LIMITED: ControlStatus.FIELD_LIMITED,
    FieldState.STOP: ControlStatus.FIELD_STOP,
    FieldState.STALE: ControlStatus.FIELD_STALE,
    FieldState.BLIND: ControlStatus.FIELD_BLIND,
}
MODE = {Mode.AUTO: ControlStatus.MODE_AUTO, Mode.REMOTE: ControlStatus.MODE_REMOTE}


def to_command(msg: Twist) -> Command:
    """Return the planar part of a Twist; a non-finite unused axis makes it non-finite."""
    unused = (msg.linear.z, msg.angular.x, msg.angular.y)
    if not all(math.isfinite(v) for v in unused):
        return Command(math.nan, math.nan, math.nan)
    return Command(msg.linear.x, msg.linear.y, msg.angular.z)


def to_twist(cmd: Command) -> Twist:
    """Return a Twist with only vx, vy and wz set."""
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = float(cmd.vx), float(cmd.vy), float(cmd.wz)
    return msg


def to_scan(msg: SectorScan) -> ScanData:
    """Return a SectorScan as plain values."""
    return ScanData(
        msg.header.frame_id,
        to_sec(msg.header.stamp),
        float(msg.angle_min),
        float(msg.angle_increment),
        float(msg.range_max),
        np.asarray(msg.ranges, dtype=float),
    )


def status_msg(
    state: ArbiterState, field: FieldState, clearance: float, speed_scale: float, stamp, frame: str
) -> ControlStatus:
    """Return control/status from the arbiter state and the newest field result."""
    msg = ControlStatus()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.mode = MODE[state.mode]
    msg.remote_source = state.remote_source
    msg.estop = state.estop
    msg.estop_reasons = list(state.estop_reasons)
    msg.armed = state.armed
    msg.field = FIELD[field]
    msg.clearance = float(clearance)
    msg.speed_scale = float(min(1.0, max(0.0, speed_scale)))
    return msg
