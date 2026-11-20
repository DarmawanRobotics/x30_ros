"""Recordings within the disk: the oldest sessions first, never the active one, links left."""

from __future__ import annotations

from datetime import datetime
import os

from x30_recorder.core.retention import prune, sessions

NOW = datetime(2026, 11, 21, 12, 0).timestamp()


def make(root, day, name):
    path = root / day / name
    (path / 'thermal').mkdir(parents=True)
    (path / 'thermal' / 'f.npz').write_bytes(b'x' * 10)
    return str(path)


def test_the_oldest_sessions_go_while_space_is_short_never_the_active_one(tmp_path):
    old = make(tmp_path, '20261119', '080000_patrol')
    mid = make(tmp_path, '20261120', '080000_patrol')
    active = make(tmp_path, '20261121', '090000_manual')
    (tmp_path / 'notes').mkdir()  # not a day: never touched

    def counting_free(root):  # 1 GB more free per session gone
        return float(3 - len(sessions(root)))

    removed, error = prune(str(tmp_path), active, NOW, 0, 2.5, counting_free)
    assert not error and removed == [old, mid]
    assert sessions(str(tmp_path)) == [active]
    assert not (tmp_path / '20261119').exists() and (tmp_path / 'notes').is_dir()


def test_expired_days_go_and_links_stay(tmp_path):
    old = make(tmp_path, '20261101', '080000_patrol')
    keep = make(tmp_path, '20261120', '080000_patrol')
    target = make(tmp_path, '20261102', '080000_patrol')
    os.symlink(target, str(tmp_path / '20261101' / '090000_patrol'))
    removed, error = prune(str(tmp_path), '', NOW, 7, 0.0, lambda _: 100.0)
    assert not error and removed == [old, target]
    assert sessions(str(tmp_path)) == [keep]
    assert os.path.islink(str(tmp_path / '20261101' / '090000_patrol'))
