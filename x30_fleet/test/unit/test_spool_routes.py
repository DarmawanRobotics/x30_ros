"""The on-disk spool of uploads and the routes NETRA keeps."""

from __future__ import annotations

from x30_fleet.core.routes import safe_route_id, summarize_csv
from x30_fleet.core.spool import Spool


def test_spool_roundtrip(tmp_path):
    """Spooled items survive until removed."""
    spool = Spool(str(tmp_path))
    uid = spool.put('inspection', {'point_id': 12}, {'thermal': b'jpg'})
    (item,) = spool.items()
    kind, meta, files = Spool.load(item)
    assert (kind, meta['uid'], files) == ('inspection', uid, {'thermal': b'jpg'})
    Spool.remove(item)
    assert spool.items() == []


def test_spool_quarantines_unreadable_items(tmp_path):
    """A corrupted item is moved aside instead of blocking every later upload."""
    spool = Spool(str(tmp_path))
    spool.put('route', {'name': 'yard'}, {'route.csv': b'x,y\n'})
    (item,) = spool.items()
    (tmp_path / item.split('/')[-1] / 'meta.json').write_text('{broken')
    try:
        Spool.load(item)
        raise AssertionError('corrupted meta.json must not load')
    except ValueError:
        spool.quarantine(item)
    assert spool.items() == []
    assert len(list((tmp_path / '.failed').iterdir())) == 1


def test_route_summary_of_an_x30_route():
    """Length, stops and loop closure from x30_navigation's CSV (header, yaw in degrees)."""
    text = 'x,y,yaw_deg,dwell_sec\n0.000,0.000,0.0,0.0\n3.000,4.000,0.0,5.0\n0.000,0.500,0.0,0.0\n'
    assert summarize_csv(text) == {'length_m': 9.61, 'points': 3, 'stops': 1, 'closed': True}


def test_route_ids_cannot_escape_the_site():
    """Only plain ids are accepted for downloaded routes."""
    for good in ('42', 42, 'route-7', 'a1b2_c3.v2'):
        assert safe_route_id(good)
    for bad in ('', None, '../etc/passwd', '/abs', 'a/b', '.hidden', 'x' * 65, ['1']):
        assert not safe_route_id(bad)
