from __future__ import annotations

import json
import os

import pytest

from x30_utils.io import (
    PathError,
    read_json,
    resolve_inside,
    unique_dir,
    valid_name,
    write_json_atomic,
)
from x30_utils.log import JsonlSink, Logger
from x30_utils.plugins import Registry, entry_points


def test_write_json_atomic_keeps_backup(tmp_path):
    path = tmp_path / 'a' / 'site.json'
    write_json_atomic(str(path), {'v': 1})
    write_json_atomic(str(path), {'v': 2})
    assert json.loads(path.read_text()) == {'v': 2}
    assert json.loads((tmp_path / 'a' / 'site.json.bak').read_text()) == {'v': 1}
    with pytest.raises(ValueError):
        write_json_atomic(str(path), {'v': float('nan')})
    assert json.loads(path.read_text()) == {'v': 2}


def test_read_json_tolerates_broken_files(tmp_path):
    path = tmp_path / 'x.json'
    assert read_json(str(path)) == (None, None)
    path.write_text('{"a": [')
    data, error = read_json(str(path))
    assert data is None and 'x.json' in error


def test_resolve_inside_blocks_traversal(tmp_path):
    root = tmp_path / 'sites'
    root.mkdir()
    assert resolve_inside(str(root), 'lobby/tags.json') == str(root / 'lobby' / 'tags.json')
    with pytest.raises(PathError):
        resolve_inside(str(root), '../etc/passwd')
    with pytest.raises(PathError):
        resolve_inside(str(root), '/etc/passwd')
    assert valid_name('lobby_2') and not valid_name('../x') and not valid_name('')


def test_unique_dir_never_collides(tmp_path):
    a, b = unique_dir(str(tmp_path)), unique_dir(str(tmp_path))
    assert a != b and os.path.isdir(a) and os.path.isdir(b)


class Base:
    pass


def test_registry():
    registry = Registry('x30.test', Base)

    @registry.register('one')
    class One(Base):
        pass

    assert registry.names() == ['one']
    assert isinstance(registry.create('one'), One)
    with pytest.raises(KeyError, match='use one of'):
        registry.get('two')
    with pytest.raises(TypeError):
        registry.register('bad')(int)


class Plugin(Base):
    pass


def test_registry_discovers_entry_points(tmp_path, monkeypatch):
    dist = tmp_path / 'x30_test_plugin-1.0.dist-info'
    dist.mkdir()
    (dist / 'METADATA').write_text('Metadata-Version: 2.1\nName: x30-test-plugin\nVersion: 1\n')
    (dist / 'entry_points.txt').write_text(
        f'[x30.test_entry_points]\nplugged = {__name__}:Plugin\n'
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    assert [e.name for e in entry_points('x30.test_entry_points')] == ['plugged']
    registry = Registry('x30.test_entry_points', Base)
    assert registry.names() == ['plugged']
    assert isinstance(registry.create('plugged'), Plugin)
    assert entry_points('x30.no_such_group') == []


class ListSink:
    level = 'debug'

    def __init__(self):
        self.records = []

    def emit(self, record):
        self.records.append(record)


def test_logger_fields_throttle_once():
    sink = ListSink()
    log = Logger('n', [sink])
    log.info('scan', points=3, age=0.12345)
    assert sink.records[0].text() == 'scan points=3 age=0.1235'
    for _ in range(3):
        log.warning('slow', throttle=10.0)
        log.error('once', once=True)
    assert [r.message for r in sink.records[1:]] == ['slow', 'once']
    log.bind(site='lobby').event('estop_engaged', reason='button')
    assert sink.records[-1].fields == {
        'site': 'lobby',
        'event': 'estop_engaged',
        'reason': 'button',
    }


def test_jsonl_sink(tmp_path):
    path = tmp_path / 'logs' / 'n.jsonl'
    sink = JsonlSink(str(path))
    Logger('n', [sink]).warning('blind', points=12)
    sink.close()
    line = json.loads(path.read_text().splitlines()[0])
    assert line['message'] == 'blind' and line['points'] == 12 and line['level'] == 'warning'
