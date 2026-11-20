"""A patrol as NETRA sees it (no ROS): started by a command, laps, alerts, finished."""

from __future__ import annotations

import time

ALERT_STATES = ('BLOCKED', 'OFF_PATH', 'FAILED')
FINISHED_STATES = ('GOAL_REACHED', 'NAV_INACTIVE')
MOVING_STATES = ('JOINING', 'FOLLOWING', 'APPROACHING', 'DWELLING')


class Patrol:
    """The patrol in progress; every method returns the messages to send to NETRA."""

    def __init__(self):
        self.current: dict | None = None  # {patrol_id, route_id}
        self.lap = 0
        self.state = 'NAV_INACTIVE'
        self.index = 0
        self.stop_index: int | None = None

    def context(self) -> dict:
        """Return the patrol context attached to events."""
        current = self.current or {}
        return {
            'patrol_id': current.get('patrol_id'),
            'route_id': current.get('route_id'),
            'ts': time.time(),
        }

    def event(self, name: str, **extra) -> dict:
        """Return a patrol_event message."""
        return {'type': 'patrol_event', 'event': name, **extra, **self.context()}

    def start(self, patrol_id, route_id) -> list[dict]:
        """Open a patrol started from NETRA."""
        self.current = {'patrol_id': patrol_id, 'route_id': route_id}
        self.lap, self.index = 0, 0
        return [self.event('started')]

    def mission(self) -> str:
        """Return the mission line of the telemetry: 'lap 2' while a loop patrol runs."""
        return f'lap {self.lap + 1}' if self.current and self.lap else ''

    def navigation(self, state: str, message: str, index: int, count: int, pose) -> list[dict]:
        """Return the alerts, laps and end of the patrol a follower status brings."""
        out = []
        previous, self.state = self.state, state
        # a lap: on a loop route the follower's waypoint index starts over
        wrapped = count > 0 and self.index >= count // 2 and index < count // 4
        if self.current and state in MOVING_STATES and wrapped:
            self.lap += 1
            out.append(self.event('lap', lap=self.lap))
        self.index = index
        if state == previous:
            return out
        if state in ALERT_STATES:
            out.append({'type': 'alert', 'state': state, 'message': message, 'pose': pose,
                        **self.context()})  # fmt: skip
        if self.current and state in FINISHED_STATES:
            out.append(self.event('finished', reason=state.lower()))
            self.current = None
        return out

    def estop(self, reasons: list[str], pose) -> dict:
        """Return the alert of an engaged e-stop."""
        text = ', '.join(reasons) or 'unknown'
        return {'type': 'alert', 'state': 'ESTOP', 'message': f'e-stop engaged ({text})',
                'pose': pose, **self.context()}  # fmt: skip

    def stop_point(self, event: str, index: int, message: str, x: float, y: float,
                   yaw_deg: float) -> dict:  # fmt: skip
        """Return the stop_event message and remember the stop being inspected."""
        self.stop_index = index if event in ('arrived', 'inspected') else None
        return {'type': 'stop_event', 'index': index, 'event': event, 'message': message,
                'x': x, 'y': y, 'yaw_deg': yaw_deg, **self.context()}  # fmt: skip
