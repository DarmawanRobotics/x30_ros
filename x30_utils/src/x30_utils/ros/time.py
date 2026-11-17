"""ROS time as seconds and back; monotonic time for watchdogs."""

from __future__ import annotations

import math
import time

import rospy

NSEC = 1_000_000_000


def monotonic() -> float:
    """Return monotonic seconds (immune to ROS and wall clock jumps)."""
    return time.monotonic()


def to_sec(stamp) -> float:
    """Return a rospy.Time (or any secs / nsecs stamp) as seconds."""
    return stamp.secs + stamp.nsecs * 1e-9


def to_stamp(seconds: float) -> rospy.Time:
    """Return seconds as rospy.Time (negative times are clamped to 0)."""
    seconds = max(0.0, float(seconds))
    sec = math.floor(seconds)
    nsec = round((seconds - sec) * 1e9)
    if nsec >= NSEC:
        sec, nsec = sec + 1, nsec - NSEC
    return rospy.Time(int(sec), int(nsec))


def stamp_age(node, stamp) -> float:
    """Return how old a header stamp is on the node's clock (negative: from the future)."""
    return node.now() - to_sec(stamp)
