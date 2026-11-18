"""World and scenario files: a path, or a name shipped in the package's config/ directory."""

from __future__ import annotations

import os

import rospkg

from x30_utils.io import expand

PACKAGE = 'x30_sim'
DEFAULT_WORLD = 'mall_loop'


def config_dir() -> str:
    """Return the package's config/ directory (source or install space)."""
    return os.path.join(rospkg.RosPack().get_path(PACKAGE), 'config')


def resolve(value: str, kind: str) -> str:
    """Return the file of a world or scenario: a path, or a name in config/<kind>/<name>.yaml."""
    value = value.strip()
    if os.sep in value or value.endswith(('.yaml', '.yml')):
        return expand(value)
    return os.path.join(config_dir(), kind, f'{value}.yaml')


def world_file(value: str) -> str:
    """Return the world file ('': the default world)."""
    return resolve(value or DEFAULT_WORLD, 'worlds')


def scenario_file(value: str) -> str:
    """Return the scenario file, '' when there is none."""
    return resolve(value, 'scenarios') if value.strip() else ''


def route_file(world_path: str) -> str:
    """Return the route shipped with a world (config/routes/<world>.csv), '' when there is none."""
    stem = os.path.splitext(os.path.basename(world_path))[0]
    path = os.path.join(config_dir(), 'routes', f'{stem}.csv')
    return path if os.path.isfile(path) else ''
