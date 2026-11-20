"""The routes the handheld lists and loads: inside the sites, CSV only."""

from __future__ import annotations

import os
import time

import pytest

from x30_bridge.core.routes import list_routes, route_path


def test_routes_are_listed_relative_and_newest_first(tmp_path):
    (tmp_path / 'gardu' / 'routes').mkdir(parents=True)
    old, new = tmp_path / 'gardu' / 'routes' / 'a.csv', tmp_path / 'gardu' / 'routes' / 'b.csv'
    old.write_text('x')
    new.write_text('x')
    os.utime(old, (time.time() - 100, time.time() - 100))
    (tmp_path / 'gardu' / 'tags.json').write_text('{}')
    assert list_routes(str(tmp_path)) == ['gardu/routes/b.csv', 'gardu/routes/a.csv']
    assert list_routes(str(tmp_path / 'missing')) == []


def test_only_csv_files_inside_the_sites(tmp_path):
    assert route_path(str(tmp_path), 'gardu/routes/a.csv') == 'gardu/routes/a.csv'
    assert route_path(str(tmp_path), str(tmp_path / 'gardu' / 'a.csv')) == 'gardu/a.csv'
    for bad in ('../x.csv', '/etc/passwd', 'gardu/tags.json', '', None):
        with pytest.raises(ValueError):
            route_path(str(tmp_path), bad)
