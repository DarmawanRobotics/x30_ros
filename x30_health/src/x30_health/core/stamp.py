"""Header stamp of a serialized ROS 1 message without deserialising it (cheap even for clouds)."""

from __future__ import annotations

import struct

_SECS_NSECS = struct.Struct('<II')  # after uint32 seq
_HEADER_TYPES = ('Header', 'std_msgs/Header')


def ros1_stamp(data: bytes) -> float | None:
    """Return the header stamp (s) of a ROS 1 message that starts with std_msgs/Header, or None."""
    if len(data) < 16:  # seq, secs, nsecs, the frame_id length
        return None
    secs, nsecs = _SECS_NSECS.unpack_from(data, 4)
    if nsecs >= 1_000_000_000:
        return None
    return secs + nsecs * 1e-9


def starts_with_header(definition: str) -> bool:
    """Return True when the first field of a message definition (connection header) is a Header."""
    for raw in definition.splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line.startswith('='):  # the nested definitions after the message's own fields
            return False
        if '=' in line:  # a constant
            continue
        parts = line.split()
        return len(parts) >= 2 and parts[0] in _HEADER_TYPES
    return False
