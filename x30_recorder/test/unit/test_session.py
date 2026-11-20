"""Sessions on disk: a folder per session, its kinds of files, a record that grows with it."""

from __future__ import annotations

from datetime import datetime
import json
import os

import pytest

from x30_recorder.core.session import Session, stamp_name

T0 = datetime(2026, 11, 21, 9, 30, 15, 250000).timestamp()


def record(session: Session) -> dict:
    with open(os.path.join(session.path, 'session.json'), encoding='utf-8') as f:
        return json.load(f)


def test_a_session_folder_per_day_and_start(tmp_path):
    s = Session(str(tmp_path), 'patrol', T0, {'site': 'gardu'})
    assert s.path == str(tmp_path / '20261121' / '093015_patrol')
    again = Session(str(tmp_path), 'patrol', T0)  # two in one second: never the same folder
    assert again.path == str(tmp_path / '20261121' / '093015_patrol-1')
    assert record(s)['reason'] == 'patrol' and record(s)['site'] == 'gardu'
    assert record(s)['stopped'] is None and record(s)['thermal_frames'] == 0
    with pytest.raises(ValueError):
        Session(str(tmp_path), 'party', T0)


def test_files_by_kind_and_a_final_record(tmp_path):
    s = Session(str(tmp_path), 'manual', T0)
    path = s.file('thermal', stamp_name(T0) + '.npz')
    assert path == os.path.join(s.path, 'thermal', '20261121_093015_250.npz')
    assert os.path.isdir(os.path.dirname(path))
    s.count('thermal_frames', 3)
    s.stop(T0 + 90.0)
    final = record(s)
    assert final['thermal_frames'] == 3 and final['stopped'] == '2026-11-21T09:31:45'
    assert s.duration(T0 + 1000.0) == pytest.approx(90.0)
