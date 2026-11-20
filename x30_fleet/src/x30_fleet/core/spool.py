"""The outbox on disk: uploads wait there until NETRA accepts them, across restarts."""

from __future__ import annotations

import json
import os
import shutil
import time
import uuid


class Spool:
    """Durable outbox on disk: each item is a directory with meta.json and attached files."""

    def __init__(self, root: str):
        self.root = os.path.expanduser(root)
        os.makedirs(self.root, exist_ok=True)

    def put(self, kind: str, meta: dict, files: dict) -> str:
        """Store an upload; return its id (also used by the server for de-duplication)."""
        uid = meta.setdefault('uid', str(uuid.uuid4()))
        tmp = os.path.join(self.root, f'.{uid}')
        os.makedirs(tmp, exist_ok=True)
        for name, data in files.items():
            with open(os.path.join(tmp, name), 'wb') as f:
                f.write(data)
        with open(os.path.join(tmp, 'meta.json'), 'w') as f:
            json.dump({'kind': kind, 'meta': meta, 'files': sorted(files)}, f)
        os.rename(tmp, os.path.join(self.root, f'{time.time():.6f}_{uid}'))
        return uid

    def items(self) -> list[str]:
        """Return pending item directories, oldest first."""
        return sorted(
            os.path.join(self.root, d) for d in os.listdir(self.root) if not d.startswith('.')
        )

    @staticmethod
    def load(item: str) -> tuple[str, dict, dict]:
        """Return (kind, meta, {name: bytes}) of an item."""
        with open(os.path.join(item, 'meta.json')) as f:
            record = json.load(f)
        files = {}
        for name in record['files']:
            with open(os.path.join(item, name), 'rb') as f:
                files[name] = f.read()
        return record['kind'], record['meta'], files

    @staticmethod
    def remove(item: str) -> None:
        """Delete an item after a successful upload."""
        shutil.rmtree(item, ignore_errors=True)

    def quarantine(self, item: str) -> None:
        """Move an unreadable item to .failed/ so it stops blocking the queue (kept for review)."""
        failed = os.path.join(self.root, '.failed')
        os.makedirs(failed, exist_ok=True)
        try:
            os.rename(item, os.path.join(failed, os.path.basename(item)))
        except OSError:
            shutil.rmtree(item, ignore_errors=True)
