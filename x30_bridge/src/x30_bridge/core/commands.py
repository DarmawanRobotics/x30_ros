"""The handheld's commands: the service each one calls, its time limit, the checks of its args."""

from __future__ import annotations

import math

# command -> std_srvs/Trigger service of the stack (relative names)
TRIGGERS = {
    'estop': 'control/estop/engage',
    'estop_release': 'control/estop/release',
    'nav_start': 'navigation/start',
    'nav_stop': 'navigation/stop',
    'nav_pause': 'navigation/pause',
    'nav_resume': 'navigation/resume',
    'record_start': 'route/start',
    'record_stop': 'route/stop',
    'localize': 'localization/start',
    'record_tags': 'localization/record_tags',
    'dock': 'docking/dock',
    'undock': 'docking/undock',
    'trip_reset': 'odometry/trip_reset',
    # the X30's own: the camera recorder (perception host) and an inspection where it stands
    'recorder_start': 'recorder/start',
    'recorder_stop': 'recorder/stop',
    'inspect': 'inspection/inspect',
}
# answered by the gateway itself, without a service
LOCAL = ('get_route', 'list_routes')
# commands that latch the software e-stop at once, before any service answers
ESTOP_COMMANDS = ('estop', 'emergency_stop')
MODES = ('auto', 'remote')
CALL_SEC = 10.0  # a service answer, at most
SLOW_CALLS = {
    'nav_start': 30.0,  # waits for a pose
    'record_tags': 30.0,
    'localize': 30.0,
    'inspect': 300.0,  # every target of the point: moves, measures, detections
}
MAX_DWELL_SEC = 3600.0


def number(value, default: float | None = None) -> float | None:
    """Return a finite int / float as float (bools and text are not numbers), else default."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return float(value)


def call_sec(name: str) -> float:
    """Return how long the service of a command may take to answer (s)."""
    return SLOW_CALLS.get(name, CALL_SEC)


def mode_of(args: dict) -> str:
    """Return the requested control mode, auto or remote."""
    mode = str(args.get('mode', 'auto')).strip().lower()
    if mode not in MODES:
        raise ValueError(f'mode must be auto or remote, got {mode[:20]!r}')
    return mode


def dwell_seconds(value) -> float:
    """Validate a stop-point dwell time from the handheld (0 < dwell <= 1 h)."""
    dwell = number(value)
    if dwell is None or not 0.0 < dwell <= MAX_DWELL_SEC:
        raise ValueError(f'dwell must be between 0 and 3600 s, got {value!r}')
    return dwell


def patrol_speed(value, low: float, high: float) -> float:
    """Validate a patrol speed from the handheld against the allowed range."""
    speed = number(value)
    if speed is None or not low <= speed <= high:
        raise ValueError(f'speed must be {low:g} to {high:g} m/s')
    return round(speed, 2)


def command_id(value):
    """Return a command id (int or str, not bool), None when it is not one."""
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        return None
    if isinstance(value, int) or (isinstance(value, str) and len(value) <= 64):
        return value
    return None


def camera_period(fps, limit: float) -> float | None:
    """Return the camera frame period for a handheld's camera_fps, capped by limit (or None)."""
    value = number(fps)
    if value is None:
        return None  # absent, text or NaN: keep the current rate
    return 1.0 / min(max(value, 0.5), max(limit, 0.1))
