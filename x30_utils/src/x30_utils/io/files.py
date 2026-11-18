"""Crash-safe file writes and tolerant reads."""

from __future__ import annotations

from datetime import datetime
import json
import os
import shutil
import tempfile


def write_text_atomic(path: str, text: str):
    """Write a file so readers see either the old or the new content, never a partial one."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp-', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    _fsync_dir(directory)


def write_json_atomic(path: str, data, backup: bool = True):
    """Write JSON atomically; keep the previous version as <path>.bak."""
    text = json.dumps(data, indent=2, sort_keys=False, allow_nan=False) + '\n'
    if backup:
        backup_file(path)
    write_text_atomic(path, text)


def read_json(path: str):
    """Return (data, error): data is None when the file is missing or broken."""
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f), None
    except FileNotFoundError:
        return None, None
    except (OSError, ValueError) as error:
        return None, f'{path}: {error}'


def backup_file(path: str) -> str | None:
    """Copy path to path.bak when it exists; return the backup path."""
    if not os.path.isfile(path):
        return None
    target = f'{path}.bak'
    shutil.copy2(path, target)
    return target


def unique_dir(parent: str, prefix: str = 'backup') -> str:
    """Create and return a new directory parent/<prefix>-<timestamp>[-n]."""
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    for n in range(1000):
        name = f'{prefix}-{stamp}' if n == 0 else f'{prefix}-{stamp}-{n}'
        path = os.path.join(parent, name)
        try:
            os.makedirs(path)
            return path
        except FileExistsError:
            continue
    raise OSError(f'no free backup directory under {parent}')


def _fsync_dir(directory: str):
    try:
        fd = os.open(directory, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)
