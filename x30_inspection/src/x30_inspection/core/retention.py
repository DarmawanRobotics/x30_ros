"""Reports kept within the disk: whole days, the oldest first, never today's (no ROS)."""

# A robot on patrol writes its reports every day; with the disk full nothing new is written,
# reports, maps and routes alike. Days older than keep_days go (0: all are kept), then the oldest
# days while less than min_free_gb is free (0: never), like a recorder that overwrites its oldest.
# Only YYYYMMDD folders of the reports directory are touched, never a link, never today's.

from __future__ import annotations

from datetime import date, timedelta
import os
import re
import shutil

DAY = re.compile(r'^\d{8}$')
GB = 1e9  # as x30_health's system:disk


def free_gb(path: str) -> float:
    """Return the free space (GB) of the file system holding path."""
    return shutil.disk_usage(path).free / GB


def report_days(root: str) -> list:
    """Return the day folders (YYYYMMDD) of a reports directory, the oldest first."""
    try:
        names = os.listdir(root)
    except OSError:
        return []
    days = []
    for name in names:
        path = os.path.join(root, name)
        if DAY.match(name) and os.path.isdir(path) and not os.path.islink(path):
            days.append(name)
    return sorted(days)


def prune(root: str, today: date, keep_days: int, min_free_gb: float, free=free_gb):
    """Remove old days of reports; return (the days removed, an error or '')."""
    root = os.path.expanduser(root)
    days = [day for day in report_days(root) if day < today.strftime('%Y%m%d')]
    expired = (today - timedelta(days=keep_days)).strftime('%Y%m%d') if keep_days > 0 else ''
    removed = []
    try:
        while days and (days[0] < expired or (min_free_gb > 0.0 and free(root) < min_free_gb)):
            shutil.rmtree(os.path.join(root, days[0]))
            removed.append(days.pop(0))
    except OSError as error:
        return removed, str(error)
    return removed, ''
