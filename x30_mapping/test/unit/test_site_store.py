"""Sites on disk: tolerant loads (#7), map backup fallback, archive uniqueness (#13)."""

from __future__ import annotations

import os

import numpy as np
import pytest

from x30_mapping.core import map_io
from x30_mapping.core.site_store import SiteStore
from x30_utils.io import PathError


def grid(origin=0.0) -> map_io.GridMap:
    data = np.zeros((10, 12), np.int8)
    data[0, :] = 100
    return map_io.GridMap(data, 0.1, origin, origin)


def site_with_everything(store: SiteStore, site='lobby') -> str:
    root = store.create(site)
    map_io.save_map(root, grid(0.0))
    map_io.save_map(root, grid(1.0))  # leaves a .bak pair
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write('{"base_map": {"x": 1, "y": 2, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}}')
    with open(os.path.join(root, 'routes', '2026-10-24_10-00-00.csv'), 'w') as f:
        f.write('x,y,yaw_deg,dwell_sec\n0,0,0,0\n')
    return root


def test_summary_of_a_complete_site(tmp_path):
    store = SiteStore(str(tmp_path))
    site_with_everything(store)
    summary = store.summary('lobby')
    assert summary.map is not None and summary.map.origin_x == 1.0
    assert summary.tags == 1
    assert summary.routes == ['2026-10-24_10-00-00.csv']
    assert summary.problems == [] and summary.has_content
    assert store.list_sites() == ['lobby']


def test_corrupt_files_never_raise(tmp_path):
    """#7: broken tags and map are reported as problems; the map falls back to the .bak."""
    store = SiteStore(str(tmp_path))
    root = site_with_everything(store)
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write('{not json')
    with open(os.path.join(root, 'map.pgm'), 'wb') as f:
        f.write(b'garbage')
    summary = store.summary('lobby')
    assert summary.tags == 0
    assert summary.map is not None and summary.map.origin_x == 0.0  # the previous map
    assert any('tags.json' in p for p in summary.problems)
    assert any('previous map' in p for p in summary.problems)
    assert store.has_content('lobby')  # an unreadable tags file is still worth archiving
    os.remove(os.path.join(root, 'map.yaml.bak'))
    assert store.summary('lobby').map is None


def test_missing_and_invalid_sites(tmp_path):
    store = SiteStore(str(tmp_path / 'none'))
    assert store.list_sites() == []
    assert store.summary('nowhere').map is None
    assert not store.exists('../etc') and not store.has_content('Bad Name')
    with pytest.raises(PathError):
        store.create('../escape')
    with pytest.raises(PathError):
        store.layout('')


def test_empty_tags_file_is_not_content(tmp_path):
    store = SiteStore(str(tmp_path))
    root = store.create('new')
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write('{}')
    assert not store.has_content('new')


def test_archive_twice_in_one_second_never_nests(tmp_path):
    """#13: every archive gets its own directory; routes never end up in routes/routes."""
    store = SiteStore(str(tmp_path))
    root = site_with_everything(store)
    first = store.archive('lobby')
    assert sorted(os.listdir(first)) == sorted(
        ['map.yaml', 'map.pgm', 'map.yaml.bak', 'map.pgm.bak', 'tags.json', 'routes']
    )
    assert os.listdir(os.path.join(root, 'routes')) == []
    assert not store.has_content('lobby')
    site_with_everything(store)
    second = store.archive('lobby')
    assert second != first
    assert os.listdir(os.path.join(second, 'routes')) == ['2026-10-24_10-00-00.csv']
    assert os.listdir(os.path.join(first, 'routes')) == ['2026-10-24_10-00-00.csv']
    assert store.archive('lobby') is None  # nothing left: no empty backup directory
    assert store.list_sites() == ['lobby']
