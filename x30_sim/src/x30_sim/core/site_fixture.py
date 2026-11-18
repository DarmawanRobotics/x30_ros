"""A ready site drawn from a world: map (PGM + YAML), tags and routes."""

from __future__ import annotations

import math
import os
import zlib

import numpy as np
import yaml

from x30_sim.core.world import World
from x30_utils.grid import GridSpec
from x30_utils.io import (
    PathError,
    RoutePoint,
    SiteLayout,
    valid_name,
    write_route,
    write_tags,
    write_text_atomic,
)

WALL_THICKNESS = 0.10  # m, drawn around every wall segment
MARGIN = 0.5  # m of unknown around the walls
FREE, OCCUPIED, UNKNOWN = 0, 100, -1
PIXELS = {FREE: 254, OCCUPIED: 0, UNKNOWN: 205}  # map_server trinary shades


def rasterize(world: World, resolution: float = 0.05, margin: float = MARGIN):
    """Return (GridSpec, data): walls and pillars occupied, inside free, else unknown."""
    xy = np.vstack([world.segments[:, 0:2], world.segments[:, 2:4]])
    if not len(xy):
        xy = np.zeros((1, 2))
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    x0, y0 = lo - margin
    width = math.ceil((hi[0] + margin - x0) / resolution)
    height = math.ceil((hi[1] + margin - y0) / resolution)
    spec = GridSpec(resolution, float(x0), float(y0), width, height)
    cols, rows = np.meshgrid(np.arange(width), np.arange(height))
    centers = spec.centers(np.column_stack((cols.ravel(), rows.ravel())))
    px, py = centers[:, 0].reshape(height, width), centers[:, 1].reshape(height, width)
    data = np.full((height, width), UNKNOWN, dtype=np.int8)
    data[(px >= lo[0]) & (px <= hi[0]) & (py >= lo[1]) & (py <= hi[1])] = FREE
    for sx0, sy0, sx1, sy1, _ in world.segments:
        dx, dy = sx1 - sx0, sy1 - sy0
        t = np.clip(((px - sx0) * dx + (py - sy0) * dy) / max(dx * dx + dy * dy, 1e-12), 0, 1)
        dist = np.hypot(px - (sx0 + t * dx), py - (sy0 + t * dy))
        data[dist <= WALL_THICKNESS / 2.0 + resolution / 2.0] = OCCUPIED
    for ox, oy, radius, _ in world.circles:
        data[np.hypot(px - ox, py - oy) <= radius] = OCCUPIED
    return spec, data


def encode_map(spec: GridSpec, data: np.ndarray) -> tuple[bytes, str]:
    """Return (PGM bytes, YAML text) of a grid for map.pgm / map.yaml."""
    image = np.full(data.shape, PIXELS[UNKNOWN], dtype=np.uint8)
    image[data == FREE] = PIXELS[FREE]
    image[data >= 65] = PIXELS[OCCUPIED]
    image = np.ascontiguousarray(np.flipud(image))  # images start at the top row
    pgm = f'P5\n# x30_sim {spec.resolution:.3f} m/pix\n{spec.width} {spec.height}\n255\n'
    pgm = pgm.encode() + image.tobytes()
    meta = {
        'image': 'map.pgm',
        'mode': 'trinary',
        'resolution': float(spec.resolution),
        'origin': [spec.origin_x, spec.origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'image_size': [spec.width, spec.height],
        'image_crc32': zlib.crc32(pgm),
    }
    return pgm, yaml.safe_dump(meta, default_flow_style=None, sort_keys=False)


def write_map(layout: SiteLayout, spec: GridSpec, data: np.ndarray):
    """Write map.pgm, then map.yaml (a reader never sees a YAML without its image)."""
    pgm, text = encode_map(spec, data)
    os.makedirs(layout.root, exist_ok=True)
    tmp = layout.map_image + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(pgm)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, layout.map_image)
    write_text_atomic(layout.map_yaml, text)


def write_site(
    world: World,
    layout: SiteLayout,
    routes: dict[str, list[RoutePoint]] | None = None,
    overwrite: bool = False,
) -> bool:
    """Write map, tags and routes of a world into a site; False when it already has a map."""
    bad = [name for name in routes or {} if not valid_name(name)]
    if bad:
        raise PathError(f'invalid route names {bad}')
    if not overwrite and os.path.isfile(layout.map_yaml):
        return False
    write_map(layout, *rasterize(world))
    write_tags(layout.tags, world.tags)
    os.makedirs(layout.routes, exist_ok=True)
    for name, points in (routes or {}).items():
        write_route(os.path.join(layout.routes, f'{name}.csv'), points)
    return True
