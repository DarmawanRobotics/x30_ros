"""Topic profiles of the stack: what ROS 1 offers of QoS (queue, latching, Nagle, buffer)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Profile:
    """How a publisher or subscriber treats its messages."""

    queue_size: int
    latch: bool = False
    tcp_nodelay: bool = False
    buff_size: int = 65536


LATCHED = Profile(1, latch=True)
"""Last value kept for late subscribers: maps, status, configuration."""

SENSOR = Profile(1, tcp_nodelay=True, buff_size=1 << 24)
"""Only the newest message, never stale ones queued in the socket: high-rate sensor data."""

COMMAND = Profile(1, tcp_nodelay=True)
"""Only the newest command matters: velocity commands, safety scans."""

EVENTS = Profile(20)
"""Every message matters: events, requests."""

RECORDED = Profile(20, tcp_nodelay=True, buff_size=1 << 24)
"""Every message of a sensor kept, none dropped while a callback runs: recordings."""


def profile(qos) -> Profile:
    """Return a Profile from a Profile or a queue size."""
    if isinstance(qos, Profile):
        return qos
    if isinstance(qos, int) and not isinstance(qos, bool) and qos >= 0:
        return Profile(qos)
    raise TypeError(f'expected a Profile or a queue size, got {qos!r}')
