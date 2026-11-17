"""Occupancy maps in the map_server format (PGM + YAML), saved crash safe with a .bak pair."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import shutil
import tempfile
import zlib

import numpy as np
import yaml

from x30_mapping.core.pgm import MapError, encode_pgm, image_to_occupancy, occupancy_to_image
from x30_mapping.core.pgm import read_image as _read_image

MAP_YAML, MAP_IMAGE = 'map.yaml', 'map.pgm'
BAK = '.bak'


@dataclass
class GridMap:
    """An occupancy grid in the map frame: data[row, col] with row 0 at origin_y."""

    data: np.ndarray  # int8 (height, width): -1 unknown, 0..100 occupancy
    resolution: float  # m per cell
    origin_x: float  # m, map position of the corner of cell (0, 0)
    origin_y: float
    origin_yaw: float = 0.0  # rad

    @property
    def width(self) -> int:
        """Return the number of columns."""
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        """Return the number of rows."""
        return int(self.data.shape[0])


def map_yaml_path(path: str) -> str:
    """Return the YAML of a map given its YAML or the directory holding map.yaml."""
    return os.path.join(path, MAP_YAML) if os.path.isdir(path) else path


def _read_meta(yaml_path: str) -> dict:
    try:
        with open(yaml_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f) or {}
    except OSError as error:
        raise MapError(f'cannot read {yaml_path}: {error}') from error
    except yaml.YAMLError as error:
        raise MapError(f'{yaml_path}: {error}') from error
    if not isinstance(meta, dict):
        raise MapError(f'{yaml_path}: expected a mapping')
    return meta


def _check_image(yaml_path: str, meta: dict, image_path: str, image: np.ndarray):
    """Compare the image with the size and CRC the YAML recorded (when it recorded them)."""
    size = meta.get('image_size')
    if size is not None and [int(v) for v in size] != [image.shape[1], image.shape[0]]:
        raise MapError(
            f'{yaml_path}: image {image_path} is {image.shape[1]}x{image.shape[0]}, '
            f'the YAML expects {size[0]}x{size[1]} (torn save?)'
        )
    crc = meta.get('image_crc32')
    if crc is not None:
        with open(image_path, 'rb') as f:
            actual = zlib.crc32(f.read())
        if int(crc) != actual:
            raise MapError(f'{yaml_path}: image {image_path} does not match its CRC (torn save?)')


def load_map(path: str) -> GridMap:
    """Load a map_server map from its YAML file (or a directory holding map.yaml)."""
    yaml_path = map_yaml_path(path)
    meta = _read_meta(yaml_path)
    try:
        image_name = str(meta['image'])
        resolution = float(meta['resolution'])
        origin = [float(v) for v in meta.get('origin', [0.0, 0.0, 0.0])]
        negate = bool(int(meta.get('negate', 0)))
        occupied = float(meta.get('occupied_thresh', 0.65))
        free = float(meta.get('free_thresh', 0.25))
        mode = str(meta.get('mode', 'trinary'))
    except (KeyError, TypeError, ValueError) as error:
        raise MapError(f'{yaml_path}: missing or bad field ({error})') from error
    if not (resolution > 0.0 and math.isfinite(resolution)) or len(origin) < 2:
        raise MapError(f'{yaml_path}: bad resolution or origin')
    if not all(math.isfinite(v) for v in origin):
        raise MapError(f'{yaml_path}: non-finite origin')
    image_path = (
        image_name
        if os.path.isabs(image_name)
        else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_name)
    )
    image = _read_image(image_path)
    try:
        _check_image(yaml_path, meta, image_path, image)
    except (OSError, TypeError, ValueError, IndexError) as error:
        if isinstance(error, MapError):
            raise
        raise MapError(f'{yaml_path}: bad image_size or image_crc32 ({error})') from error
    data = image_to_occupancy(image, negate, occupied, free, mode)
    yaw = origin[2] if len(origin) > 2 else 0.0
    return GridMap(data, resolution, origin[0], origin[1], yaw)


def _meta(grid: GridMap, image_name: str, pgm: bytes, image: np.ndarray) -> dict:
    return {
        'image': image_name,
        'mode': 'trinary',
        'resolution': float(grid.resolution),
        'origin': [float(grid.origin_x), float(grid.origin_y), float(grid.origin_yaw)],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,  # unknown (205) stays unknown in every map_server
        'image_size': [int(image.shape[1]), int(image.shape[0])],
        'image_crc32': zlib.crc32(pgm),
    }


def _write_synced(path: str, data: bytes):
    with open(path, 'wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


def _fsync_dir(directory: str):
    try:
        fd = os.open(directory, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def _backup_pair(directory: str, staging: str):
    """Keep the current pair as map.pgm.bak + map.yaml.bak (the YAML pointing at the .bak)."""
    yaml_path, image_path = (os.path.join(directory, n) for n in (MAP_YAML, MAP_IMAGE))
    if not os.path.isfile(yaml_path):
        return
    try:
        meta = _read_meta(yaml_path)
    except MapError:
        meta = None
    if os.path.isfile(image_path):
        bak_image = os.path.join(staging, MAP_IMAGE + BAK)
        shutil.copyfile(image_path, bak_image)
        os.replace(bak_image, image_path + BAK)
    bak_yaml = os.path.join(staging, MAP_YAML + BAK)
    if meta is not None and str(meta.get('image', '')) == MAP_IMAGE:
        meta['image'] = MAP_IMAGE + BAK
        text = yaml.safe_dump(meta, default_flow_style=None, sort_keys=False).encode()
        _write_synced(bak_yaml, text)
    else:
        shutil.copyfile(yaml_path, bak_yaml)
    os.replace(bak_yaml, yaml_path + BAK)


def save_map(directory: str, grid: GridMap) -> str:
    """Write map.pgm + map.yaml (trinary) into directory, crash safe; return the YAML path."""
    os.makedirs(directory, exist_ok=True)
    image = occupancy_to_image(grid.data)
    pgm = encode_pgm(image, f'x30_mapping {grid.resolution:.3f} m/pix')
    meta = _meta(grid, MAP_IMAGE, pgm, image)
    text = yaml.safe_dump(meta, default_flow_style=None, sort_keys=False).encode()
    staging = tempfile.mkdtemp(prefix='.map-tmp-', dir=directory)
    try:
        _write_synced(os.path.join(staging, MAP_IMAGE), pgm)
        _write_synced(os.path.join(staging, MAP_YAML), text)
        _backup_pair(directory, staging)
        os.replace(os.path.join(staging, MAP_IMAGE), os.path.join(directory, MAP_IMAGE))
        os.replace(os.path.join(staging, MAP_YAML), os.path.join(directory, MAP_YAML))
        _fsync_dir(directory)
    finally:
        shutil.rmtree(staging, ignore_errors=True)
    return os.path.join(directory, MAP_YAML)


def known_bounds(data: np.ndarray) -> tuple[int, int, int, int] | None:
    """Return (row0, row1, col0, col1) of the known cells (inclusive), or None if none."""
    rows = np.flatnonzero((data >= 0).any(axis=1))
    cols = np.flatnonzero((data >= 0).any(axis=0))
    if rows.size == 0:
        return None
    return int(rows[0]), int(rows[-1]), int(cols[0]), int(cols[-1])


def crop_known(grid: GridMap, margin_m: float) -> GridMap:
    """Return the grid cut to its known cells plus a margin (unchanged when nothing is known)."""
    bounds = known_bounds(grid.data)
    if bounds is None:
        return grid
    pad = math.ceil(max(margin_m, 0.0) / grid.resolution)
    r0, r1, c0, c1 = bounds
    r0, c0 = max(r0 - pad, 0), max(c0 - pad, 0)
    r1, c1 = min(r1 + pad, grid.height - 1), min(c1 + pad, grid.width - 1)
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = c0 * grid.resolution, r0 * grid.resolution
    return GridMap(
        grid.data[r0 : r1 + 1, c0 : c1 + 1].copy(),
        grid.resolution,
        grid.origin_x + c * dx - s * dy,
        grid.origin_y + s * dx + c * dy,
        grid.origin_yaw,
    )
