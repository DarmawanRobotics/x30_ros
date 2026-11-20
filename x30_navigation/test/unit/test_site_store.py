"""Sites on disk: folders of tags and routes, loaded tolerantly (#7), never a map."""

from __future__ import annotations

import os

import pytest

from x30_navigation.core.config.sites import SiteConfig
from x30_navigation.core.sites.store import SiteStore
from x30_utils.config import build
from x30_utils.io import PathError


def site_with_everything(store: SiteStore, site='lobby') -> str:
    root = store.create(site)
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write('{"base_map": {"x": 1, "y": 2, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}}')
    with open(os.path.join(root, 'routes', '2026-10-24_10-00-00.csv'), 'w') as f:
        f.write('x,y,yaw_deg,dwell_sec\n0,0,0,0\n')
    return root


def test_summary_of_a_complete_site(tmp_path):
    store = SiteStore(str(tmp_path))
    site_with_everything(store)
    summary = store.summary('lobby')
    assert summary.tags == 1
    assert summary.routes == ['2026-10-24_10-00-00.csv']
    assert summary.problems == []
    assert store.list_sites() == ['lobby']


def test_corrupt_files_never_raise(tmp_path):
    """#7: a broken tags file is reported as a problem."""
    store = SiteStore(str(tmp_path))
    root = site_with_everything(store)
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write('{not json')
    summary = store.summary('lobby')
    assert summary.tags == 0 and summary.routes == ['2026-10-24_10-00-00.csv']
    assert any('tags.json' in p for p in summary.problems)


def test_missing_and_invalid_sites(tmp_path):
    store = SiteStore(str(tmp_path / 'none'))
    assert store.list_sites() == []
    summary = store.summary('nowhere')
    assert (summary.tags, summary.routes, summary.problems) == (0, [], [])
    assert not store.exists('../etc') and not store.exists('Bad Name')
    with pytest.raises(PathError):
        store.create('../escape')
    with pytest.raises(PathError):
        store.layout('')


def test_a_new_site_is_an_empty_folder_for_routes(tmp_path):
    store = SiteStore(str(tmp_path))
    root = store.create('new')
    assert os.listdir(root) == ['routes'] and store.exists('new')
    assert store.summary('new').routes == []


def test_the_launch_site_is_created_by_default():
    assert build(SiteConfig, {}).create_site
