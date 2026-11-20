#!/usr/bin/env python3
"""x30_logs: ROS's log directory kept within a budget, the oldest runs first (x30_host.sh start).

roslaunch writes a directory per run into ~/.ros/log (ROS_LOG_DIR, ROS_HOME/log): rosout.log up
to 1 GB, each rospy node up to 250 MB. A robot whose watchdog restarts its stack for months fills
its disk with them. Before the stack starts, whole run directories go, the oldest first, until
the rest fits the budget; the newest --keep runs always stay. Standard library only.
"""

from __future__ import annotations

import argparse
import contextlib
import os
from pathlib import Path
import shutil
import sys

MB = 1024 * 1024


def log_dir() -> Path:
    """Return the directory roslaunch writes its logs to."""
    if os.environ.get('ROS_LOG_DIR'):
        return Path(os.environ['ROS_LOG_DIR'])
    home = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    return Path(home) / 'log'


def size_of(path: Path) -> int:
    """Return the bytes of the files under path (links not followed)."""
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            with contextlib.suppress(OSError):  # gone meanwhile
                total += os.lstat(os.path.join(root, name)).st_size
    return total


def prune(root: Path, budget: int, keep: int) -> tuple[list, int]:
    """Remove the oldest run directories until root fits budget; return (removed, bytes left)."""
    try:
        entries = list(root.iterdir())
    except OSError:
        return [], 0
    runs = sorted(
        (e for e in entries if e.is_dir() and not e.is_symlink()),
        key=lambda e: e.stat().st_mtime,
    )
    loose = sum(e.lstat().st_size for e in entries if not e.is_dir() or e.is_symlink())
    sizes = {run: size_of(run) for run in runs}
    total = loose + sum(sizes.values())
    removed = []
    for run in runs[: max(len(runs) - keep, 0)]:
        if total <= budget:
            break
        shutil.rmtree(run, ignore_errors=True)
        if not run.exists():
            removed.append(run.name)
            total -= sizes[run]
    return removed, total


def main(argv: list | None = None) -> int:
    """Prune and say what went (nothing said when nothing did)."""
    parser = argparse.ArgumentParser(prog='x30_logs', description=__doc__.splitlines()[0])
    parser.add_argument('--dir', type=Path, default=None, help='default: where roslaunch logs')
    parser.add_argument('--budget-mb', type=float, default=2048.0)
    parser.add_argument('--keep', type=int, default=5, help='newest runs never removed')
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    root = args.dir or log_dir()
    removed, left = prune(root, int(args.budget_mb * MB), args.keep)
    if removed:
        print(f'ROS logs: {len(removed)} old runs removed, {left / MB:.0f} MB left in {root}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
