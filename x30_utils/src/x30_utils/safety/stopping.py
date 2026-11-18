"""One braking model for every speed limit: d = v t_r + v^2 / 2a."""

from __future__ import annotations

import math


def stopping_distance(speed: float, decel: float, reaction_time: float) -> float:
    """Distance covered from a command at speed to standstill."""
    v = abs(speed)
    return v * reaction_time + v * v / (2.0 * decel)


def stopping_speed(room: float, decel: float, reaction_time: float) -> float:
    """Highest speed that still stops within room (m); 0 without room, inf with infinite room."""
    if not room > 0.0:
        return 0.0
    if math.isinf(room):
        return math.inf
    t = reaction_time
    return decel * (-t + math.sqrt(t * t + 2.0 * room / decel))


def stopping_yaw_rate(room: float, yaw_decel: float, reaction_time: float) -> float:
    """Highest yaw rate that still stops within room (rad)."""
    return stopping_speed(room, yaw_decel, reaction_time)
