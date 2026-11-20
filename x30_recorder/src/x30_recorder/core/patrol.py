"""When a patrol runs: the path follower on a route, with a grace for laps and stop points."""

from __future__ import annotations

import math

# x30_msgs/NavigationStatus states that mean a route is running (paused and blocked too)
RUNNING = {1, 2, 3, 4, 5, 6, 7}  # JOINING ... OFF_PATH; INACTIVE 0, GOAL_REACHED 8, FAILED 9


class Patrol:
    """Whether an automatic session should run, from the follower's last state over time."""

    def __init__(self, stop_after_sec: float, silence_sec: float):
        self.stop_after_sec, self.silence_sec = stop_after_sec, silence_sec
        self.heard = -math.inf  # the last status
        self.running = False
        self.ended_at = -math.inf  # when the route stopped running

    def update(self, state: int, now: float):
        """Take a navigation status."""
        running = state in RUNNING
        if self.running and not running:
            self.ended_at = now
        self.running, self.heard = running, now

    def wanted(self, now: float) -> bool:
        """Return True while a route runs and stop_after_sec after (unless status went silent)."""
        if now - self.heard > self.silence_sec:
            return False
        return self.running or now - self.ended_at <= self.stop_after_sec
