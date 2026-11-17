from __future__ import annotations

import math

import pytest

from x30_utils.io import (
    PathError,
    RouteError,
    RoutePoint,
    SiteLayout,
    list_sites,
    read_route,
    read_tags,
    write_route,
    write_tags,
)


def test_route_roundtrip_and_legacy_columns(tmp_path):
    path = tmp_path / 'r.csv'
    points = [RoutePoint(0.0, 1.0, math.radians(90)), RoutePoint(2.0, 1.0, 0.0, 5.0)]
    write_route(str(path), points)
    back = read_route(str(path))
    assert back[0].yaw == pytest.approx(math.pi / 2)
    assert back[1].is_stop
    legacy = tmp_path / 'legacy.csv'
    legacy.write_text('x,y,yaw,dwell_sec\n1,2,0.5,0\n')
    assert read_route(str(legacy))[0].yaw == 0.5
    legacy.write_text('x,y,yaw_deg\n1,nan,0\n')
    with pytest.raises(RouteError):
        read_route(str(legacy))


def test_site_layout_and_tags(tmp_path):
    layout = SiteLayout(str(tmp_path), 'lobby')
    assert layout.tags.endswith('lobby/tags.json')
    with pytest.raises(PathError):
        SiteLayout(str(tmp_path), '../etc')
    write_tags(layout.tags, {'base_map': (1, 2, 0, 0, 0, 0, 1)})
    (tmp_path / 'not a site').mkdir()
    assert list_sites(str(tmp_path)) == ['lobby']
    tags, problems = read_tags(layout.tags)
    assert tags == {'base_map': (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)}
    assert not problems
    (tmp_path / 'lobby' / 'tags.json').write_text(
        '{"a": {"x": 1}, "b": {"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1}}'
    )
    tags, problems = read_tags(layout.tags)
    assert list(tags) == ['b']
    assert len(problems) == 1
    (tmp_path / 'lobby' / 'tags.json').write_text('{"a": [')
    tags, problems = read_tags(layout.tags)
    assert tags == {}
    assert problems


def test_write_tags_rejects_wrong_length_and_keeps_the_file(tmp_path):
    path = str(tmp_path / 'tags.json')
    write_tags(path, {'a': (0, 0, 0, 0, 0, 0, 1)})
    with pytest.raises(ValueError, match='expected 7 values'):
        write_tags(path, {'a': (0, 0, 0, 0, 0, 0, 1), 'b': (1, 2, 3)})
    assert list(read_tags(path)[0]) == ['a']
