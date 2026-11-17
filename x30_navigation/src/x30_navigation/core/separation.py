"""Speed and separation monitoring (ISO/TS 15066): the braking model plus a moving person."""

# Kept identical in x30_navigation and x30_control: the guard and the field share it.

from __future__ import annotations

import math


def protective_distance(speed: float, decel: float, reaction_time: float, human_speed: float):
    """Return the distance (m) the robot at speed and an approaching person close until stop."""
    v = abs(speed)
    stop_time = reaction_time + v / decel
    return v * reaction_time + v * v / (2.0 * decel) + human_speed * stop_time


def separation_speed(room: float, decel: float, reaction_time: float, human_speed: float):
    """Return the highest speed (m/s) whose protective distance fits room (m); 0 without room."""
    if not room > 0.0:
        return 0.0
    if math.isinf(room):
        return math.inf
    rest = room - human_speed * reaction_time
    if rest <= 0.0:
        return 0.0
    b = reaction_time + human_speed / decel
    return decel * (-b + math.sqrt(b * b + 2.0 * rest / decel))
