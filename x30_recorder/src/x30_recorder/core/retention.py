"""Recordings within the disk: the oldest sessions go first, never the one recording (no ROS)."""

# The perception host's disk is shared with Deep Robotics' own software: it must never fill up.
# Sessions older than keep_days go (0: kept), then the oldest while less than min_free_gb is free.
# Only <YYYYMMDD>/<HHMMSS>_<reason> folders are touched, never a link, never the active one; a day
# folder left empty goes too.

from __future__ import annotations

from datetime import datetime, timedelta
import os
import re
import shutil

DAY = re.compile(r'^\d{8}$')
SESSION = re.compile(r'^\d{6}_[a-z]+(-\d+)?$')
GB = 1e9  # as x30_health's system:disk


def free_gb(path: str) -> float:
    """Return the free space (GB) of the file system holding path."""
    return shutil.disk_usage(path).free / GB


def sessions(root: str) -> list[str]:
    """Return the session folders under root, the oldest first."""
    found = []
    for day in _dirs(root, DAY):
        found += [os.path.join(root, day, s) for s in _dirs(os.path.join(root, day), SESSION)]
    return found


def prune(root: str, active: str, now: float, keep_days: int, min_free_gb: float, free=free_gb):
    """Remove old sessions; return (the sessions removed, an error or '')."""
    root = os.path.expanduser(root)
    if not os.path.isdir(root):
        return [], ''
    expired = ''
    if keep_days > 0:
        expired = (datetime.fromtimestamp(now) - timedelta(days=keep_days)).strftime('%Y%m%d')
    keep = os.path.abspath(active) if active else ''
    removed = []
    try:
        for path in sessions(root):
            if os.path.abspath(path) == keep:
                continue
            day = os.path.basename(os.path.dirname(path))
            if not (day < expired or (min_free_gb > 0.0 and free(root) < min_free_gb)):
                break
            shutil.rmtree(path)
            removed.append(path)
            parent = os.path.dirname(path)
            if not os.listdir(parent):
                os.rmdir(parent)
    except OSError as error:
        return removed, str(error)
    return removed, ''


def _dirs(parent: str, pattern) -> list[str]:
    try:
        names = os.listdir(parent)
    except OSError:
        return []
    return sorted(
        n
        for n in names
        if pattern.match(n)
        and os.path.isdir(os.path.join(parent, n))
        and not os.path.islink(os.path.join(parent, n))
    )
