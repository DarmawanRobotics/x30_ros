"""draw_site: write a site (tags, the world's route) from a world; print its directory."""

from __future__ import annotations

import argparse
import os
import sys

from x30_sim.core.site_fixture import write_site
from x30_sim.core.world import World, WorldError
from x30_sim.utils.files import route_file, world_file
from x30_utils.io import PathError, SiteLayout, read_route
from x30_utils.io.routes import RouteError

SITES_DIR = '/tmp/x30_sim/sites'
INVALID = 2


def parse(argv: list[str] | None) -> argparse.Namespace:
    """Return the command line arguments."""
    parser = argparse.ArgumentParser(
        prog='draw_site',
        description="Draw a site from a world (its tags and route, in the frame of the robot's "
        'start), unless it has its tags already. Prints the site path (roslaunch: '
        '<param command="rosrun x30_sim draw_site ..."/> runs it before any node).',
    )
    parser.add_argument('--world', default='', help='file or name in config/worlds (mall_loop)')
    parser.add_argument('--site', required=True, help='site name')
    parser.add_argument('--sites-dir', default=SITES_DIR, help=f'default {SITES_DIR}')
    parser.add_argument('--overwrite', action='store_true', help='replace its tags and route')
    return parser.parse_args(argv)


def draw(world: str, site: str, sites_dir: str, overwrite: bool = False) -> tuple[str, bool]:
    """Write the site of a world; return its directory and whether it was (re)drawn."""
    path = world_file(world)
    layout = SiteLayout(sites_dir, site)
    route = route_file(path)
    stem = os.path.splitext(os.path.basename(path))[0]
    routes = {stem: read_route(route)} if route else {}
    return layout.root, write_site(World.from_yaml(path), layout, routes, overwrite)


def main(argv: list[str] | None = None) -> int:
    """Run draw_site; exit 2 on a bad world, site name or route."""
    args = parse(argv)
    try:
        root, drawn = draw(args.world, args.site, args.sites_dir, args.overwrite)
    except (WorldError, PathError, RouteError, OSError) as error:
        print(f'draw_site: {error}', file=sys.stderr)
        return INVALID
    print(root)
    print(f'draw_site: {"drew" if drawn else "kept (it has its tags)"} {root}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
