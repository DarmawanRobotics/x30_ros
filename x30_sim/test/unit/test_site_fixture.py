"""Site fixture: the world as a map, tags and a route, loadable through x30_utils.io."""

from __future__ import annotations

import math
from pathlib import Path
import zlib

import numpy as np
import pytest
import yaml

from x30_sim.core.site_fixture import rasterize, write_site
from x30_sim.core.world import World
from x30_utils.io import PathError, SiteLayout, read_route, read_tags

ROOT = Path(__file__).resolve().parents[2]
WORLD = World.from_yaml(str(ROOT / 'config' / 'worlds' / 'mall_loop.yaml'))
ROUTE = read_route(str(ROOT / 'config' / 'routes' / 'mall_loop.csv'))


def cell(spec, data, x, y) -> int:
    """Return the occupancy value at a map position."""
    col, row = spec.cells(np.array([[x, y]]))[0]
    return int(data[row, col])


def as_array(points) -> np.ndarray:
    """Return route points as rows (x, y, yaw, dwell_sec)."""
    return np.array([(p.x, p.y, p.yaw, p.dwell_sec) for p in points])


def read_pgm(path: Path) -> np.ndarray:
    """Decode a binary PGM written by the fixture (rows top first)."""
    raw = path.read_bytes()
    lines = raw.split(b'\n', 4)
    assert lines[0] == b'P5' and lines[1].startswith(b'#') and lines[3] == b'255'
    width, height = (int(v) for v in lines[2].split())
    return np.frombuffer(lines[4], dtype=np.uint8).reshape(height, width)


def test_rasterized_world_has_walls_corridors_and_pillars():
    """Walls and pillars are occupied, the corridor is free, outside the walls is unknown."""
    spec, data = rasterize(WORLD)
    assert cell(spec, data, 6.0, 0.0) == 100  # outer wall
    assert cell(spec, data, 0.0, -1.5) == 100  # shop block
    assert cell(spec, data, 5.3, 3.8) == 100  # pillar
    assert cell(spec, data, -4.5, -3.0) == 0  # the start, in the corridor
    assert cell(spec, data, 6.3, 0.0) == -1  # beyond the outer wall


def test_route_runs_on_free_cells():
    """Every waypoint of the shipped route is on a free cell, 0.3 m clear of walls."""
    spec, data = rasterize(WORLD)
    for p in ROUTE:
        for dx, dy in ((0, 0), (0.3, 0), (-0.3, 0), (0, 0.3), (0, -0.3)):
            assert cell(spec, data, p.x + dx, p.y + dy) == 0, p
    assert sum(p.is_stop for p in ROUTE) == 1


def test_written_site_loads(tmp_path):
    """Map (map_server PGM + YAML with size and CRC), tags and route are readable."""
    layout = SiteLayout(str(tmp_path), 'mall')
    assert write_site(WORLD, layout, {'loop': ROUTE})
    meta = yaml.safe_load(Path(layout.map_yaml).read_text())
    assert meta['image'] == 'map.pgm' and meta['mode'] == 'trinary' and meta['negate'] == 0
    assert math.isclose(meta['resolution'], 0.05)
    image_bytes = Path(layout.map_image).read_bytes()
    assert meta['image_crc32'] == zlib.crc32(image_bytes)
    image = read_pgm(Path(layout.map_image))
    assert meta['image_size'] == [image.shape[1], image.shape[0]]
    spec, data = rasterize(WORLD)
    assert np.array_equal(np.flipud(image) == 0, data == 100)  # occupied black, row 0 at bottom
    assert meta['origin'][:2] == pytest.approx([spec.origin_x, spec.origin_y])
    tags, problems = read_tags(layout.tags)
    assert not problems and set(tags) == {'base_map', 'charging_dock'}
    written = read_route(str(Path(layout.routes) / 'loop.csv'))
    assert as_array(written) == pytest.approx(as_array(ROUTE), abs=1e-3)
    assert not (Path(layout.root) / 'pois.json').exists()
    assert not (Path(layout.root) / 'tours.json').exists()


def test_site_is_written_once(tmp_path):
    """An existing map is left alone unless overwrite is set."""
    layout = SiteLayout(str(tmp_path), 'mall')
    assert write_site(WORLD, layout)
    Path(layout.tags).write_text('{}')
    assert not write_site(WORLD, layout)
    assert Path(layout.tags).read_text() == '{}'
    assert write_site(WORLD, layout, overwrite=True)
    assert 'base_map' in read_tags(layout.tags)[0]


def test_bad_route_name_is_refused(tmp_path):
    """Route names follow the site naming rules (no path traversal)."""
    with pytest.raises(PathError):
        write_site(WORLD, SiteLayout(str(tmp_path), 'mall'), {'../evil': ROUTE})


def test_x30_mapping_site_store_reads_the_site(tmp_path):
    """Cross-check with x30_mapping's SiteStore (as the mapping node reads), if installed."""
    site_store = pytest.importorskip('x30_mapping.core.site_store')
    store = site_store.SiteStore(str(tmp_path))
    write_site(WORLD, store.layout('mall'), {'loop': ROUTE})
    grid, problems = store.load_map('mall')
    assert grid is not None and not problems
    _, data = rasterize(WORLD)
    assert np.array_equal(grid.data == 100, data == 100)
    assert store.tags('mall') == (2, [])
    assert store.routes('mall') == ['loop.csv']
