"""Where a site lives on disk: <site>/map.yaml + map.pgm, tags.json, routes/*.csv."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os

from x30_utils.io.files import read_json, write_json_atomic
from x30_utils.io.paths import PathError, expand, valid_name

TAG_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


@dataclass(frozen=True)
class SiteLayout:
    """Paths of one site."""

    sites_dir: str
    site: str

    def __post_init__(self):
        if not valid_name(self.site):
            raise PathError(f'invalid site name {self.site!r}')

    @property
    def root(self) -> str:
        """The site directory."""
        return os.path.join(expand(self.sites_dir), self.site)

    @property
    def map_yaml(self) -> str:
        """The map metadata file."""
        return os.path.join(self.root, 'map.yaml')

    @property
    def map_image(self) -> str:
        """The map image."""
        return os.path.join(self.root, 'map.pgm')

    @property
    def tags(self) -> str:
        """The fiducial file."""
        return os.path.join(self.root, 'tags.json')

    @property
    def routes(self) -> str:
        """The routes directory."""
        return os.path.join(self.root, 'routes')


def list_sites(sites_dir: str) -> list[str]:
    """Return the valid site names under sites_dir."""
    root = expand(sites_dir)
    if not os.path.isdir(root):
        return []
    return sorted(n for n in os.listdir(root) if valid_name(n) and os.path.isdir(f'{root}/{n}'))


def read_tags(path: str) -> tuple[dict[str, tuple], list[str]]:
    """Return ({frame: (x, y, z, qx, qy, qz, qw)}, problems); broken entries are skipped."""
    data, error = read_json(path)
    if error:
        return {}, [error]
    if data is None:
        return {}, []
    if not isinstance(data, dict):
        return {}, [f'{path}: expected an object of tag frames']
    tags, problems = {}, []
    for frame, value in data.items():
        try:
            pose = tuple(float(value[k]) for k in TAG_KEYS)
        except (KeyError, TypeError, ValueError):
            problems.append(f'{path}: tag {frame!r} skipped (needs {", ".join(TAG_KEYS)})')
            continue
        if not all(math.isfinite(v) for v in pose):
            problems.append(f'{path}: tag {frame!r} skipped (non-finite)')
            continue
        tags[str(frame)] = pose
    return tags, problems


def write_tags(path: str, tags: dict[str, tuple]):
    """Write fiducial poses atomically, keeping the previous file as .bak."""
    data = {}
    for frame, pose in tags.items():
        if len(pose) != len(TAG_KEYS):
            raise ValueError(f'tag {frame!r}: expected {len(TAG_KEYS)} values, got {len(pose)}')
        data[frame] = {k: round(float(v), 6) for k, v in zip(TAG_KEYS, pose)}
    write_json_atomic(path, data)
