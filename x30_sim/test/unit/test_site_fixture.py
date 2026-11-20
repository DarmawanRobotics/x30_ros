"""Site fixture: the world's tags and route in the frame of the robot's start, never a map."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from x30_sim.core.site_fixture import in_start_frame, site_route, site_tags, write_site
from x30_sim.core.world import World
from x30_utils.io import PathError, RoutePoint, SiteLayout, read_route, read_tags
from x30_utils.math import point_segment_distance

ROOT = Path(__file__).resolve().parents[2]
WORLD = World.from_yaml(str(ROOT / 'config' / 'worlds' / 'mall_loop.yaml'))
ROUTE = read_route(str(ROOT / 'config' / 'routes' / 'mall_loop.csv'))


def as_array(points) -> np.ndarray:
    """Return route points as rows (x, y, yaw, dwell_sec)."""
    return np.array([(p.x, p.y, p.yaw, p.dwell_sec) for p in points])


def clearance(world: World, x: float, y: float) -> float:
    """Return the distance (m) from a world position to the nearest wall or pillar."""
    walls = [point_segment_distance((x, y), s[:2], s[2:4]) for s in world.segments]
    pillars = [math.hypot(x - c[0], y - c[1]) - c[2] for c in world.circles]
    return min(walls + pillars)


def test_the_route_runs_clear_of_the_walls():
    """Every waypoint of the shipped route is 0.3 m clear of walls and pillars."""
    for p in ROUTE:
        assert clearance(WORLD, p.x, p.y) >= 0.3, p
    assert sum(p.is_stop for p in ROUTE) == 1


def test_the_start_is_the_origin_and_poses_turn_with_it():
    """The robot's start is the map origin, as the odometry's start is on the robot."""
    assert in_start_frame(WORLD, WORLD.start).to_array() == pytest.approx([0.0, 0.0, 0.0])
    world = World.from_dict({'start': [1.0, 2.0, 90.0], 'tags': {'t': [1.0, 3.0, 0.5, 90.0]}})
    ahead = World.from_dict({'tags': {'t': [1.0, 0.0, 0.5, 0.0]}}).tags['t']
    assert site_tags(world)['t'] == pytest.approx(ahead)  # 1 m ahead, facing the same way
    route = site_route(world, [RoutePoint(1.0, 3.0, math.pi / 2.0, 5.0)])
    assert as_array(route)[0] == pytest.approx([1.0, 0.0, 0.0, 5.0])


def test_written_site_loads(tmp_path):
    """Tags and route, in the start's frame, readable; no map."""
    layout = SiteLayout(str(tmp_path), 'mall')
    assert write_site(WORLD, layout, {'loop': ROUTE})
    tags, problems = read_tags(layout.tags)
    assert not problems and set(tags) == {'base_map', 'charging_dock'}
    written = read_route(str(Path(layout.routes) / 'loop.csv'))
    assert as_array(written) == pytest.approx(as_array(site_route(WORLD, ROUTE)), abs=1e-3)
    assert as_array(written)[0, :3] == pytest.approx([0.0, 0.0, 0.0], abs=1e-3)  # the start
    assert sorted(p.name for p in Path(layout.root).iterdir()) == ['routes', 'tags.json']


def test_site_is_written_once(tmp_path):
    """A site with its tags is left alone unless overwrite is set."""
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
    """Cross-check with x30_mapping's SiteStore (as the site manager reads), if installed."""
    site_store = pytest.importorskip('x30_mapping.core.site_store')
    store = site_store.SiteStore(str(tmp_path))
    write_site(WORLD, store.layout('mall'), {'loop': ROUTE})
    assert store.tags('mall') == (2, [])
    assert store.routes('mall') == ['loop.csv']
