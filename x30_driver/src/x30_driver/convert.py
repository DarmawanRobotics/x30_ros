"""Conversions between the motion host's reports and ROS conventions (no ROS imports)."""

import math
from typing import Iterable, Optional, Tuple

from x30_driver import protocol as p


def quaternion_from_rpy(roll: float, pitch: float,
                        yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) of the ZYX Euler angles in radians."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def attitude(sensor: Optional[p.SensorData]) -> Tuple[float, float, float]:
    """Return the IMU roll, pitch and yaw in radians (the report carries degrees)."""
    if sensor is None:
        return 0.0, 0.0, 0.0
    return tuple(math.radians(v) for v in sensor.rpy_deg)


def odometry_orientation(motion: p.MotionState,
                         sensor: Optional[p.SensorData]) -> Tuple[float, float, float, float]:
    """Return the leg odometry orientation: IMU roll and pitch, odometry yaw."""
    roll, pitch, _ = attitude(sensor)
    return quaternion_from_rpy(roll, pitch, motion.leg_odom_pos[2])


def finite_or(value: Optional[float], default: float) -> float:
    """Return the value when it is a finite number, the default otherwise."""
    if value is None or not math.isfinite(value):
        return default
    return float(value)


def battery_voltage(raw: int, scale: float = 0.0) -> float:
    """
    Return the battery voltage in volts.

    The document gives uint16 volts. With scale 0 the unit is guessed from the size of the value
    (no robot battery is above 200 V): above 10000 millivolts, above 200 tens of millivolts.
    """
    if scale > 0.0:
        return raw * scale
    if raw > 10000:
        return raw * 0.001
    if raw > 200:
        return raw * 0.01
    return float(raw)


def max_finite(values: Iterable[float], default: float = float('nan')) -> float:
    """Return the largest finite value, the default when there is none."""
    finite = [float(v) for v in values if v is not None and math.isfinite(v)]
    return max(finite) if finite else default


def battery_fraction(level: int) -> float:
    """Return the battery level (%) as 0-1."""
    return max(0.0, min(1.0, level / 100.0))
