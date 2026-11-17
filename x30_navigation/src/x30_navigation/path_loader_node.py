"""path_loader_node: load taught routes (CSV inside the sites directory) into the follower."""

from __future__ import annotations

import os

from nav_msgs.msg import Path

from x30_msgs.msg import WaypointPath
from x30_msgs.srv import LoadPath
from x30_navigation.core.config.loader import MAX_POINTS, LoaderConfig
from x30_navigation.core.path.processing import process
from x30_navigation.utils.conversions import nav_path, waypoint_path
from x30_utils.io import (
    PathError,
    RouteError,
    expand,
    read_route,
    resolve_inside,
    write_text_atomic,
)
from x30_utils.ros import X30Node, run

NAME = 'path_loader_node'


class PathLoaderNode(X30Node):
    """navigation/load_path: read and process a route, publish it latched for the follower."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(LoaderConfig)
        self.robot = self.declare_robot()
        self.route, self.raw, self.source = [], [], ''
        self.path_pub = self.latched(Path, 'navigation/path')
        self.raw_pub = self.latched(Path, 'navigation/path_raw')
        self.waypoints_pub = self.latched(WaypointPath, 'navigation/waypoints')
        self.create_service(LoadPath, 'navigation/load_path', self.on_load)
        if self.config.republish_sec > 0.0:
            self.every(self.config.republish_sec, self.publish_paths)
        start = self.config.route or self.remembered()
        if start:
            ok, message = self.load(start)
            self.log.info('route at start', ok=ok, detail=message)

    def remembered(self) -> str:
        """Return the route loaded last (survives restarts), or ''."""
        path = expand(self.config.remember_file)
        try:
            with open(path, encoding='utf-8') as f:
                return f.read().strip()
        except OSError:
            return ''

    def on_load(self, request, response):
        """Service: load a route file inside sites_dir."""
        response.success, response.message = self.load(request.path)
        if response.success and self.config.remember_file:
            try:
                write_text_atomic(expand(self.config.remember_file), self.source + '\n')
            except OSError as error:
                self.log.warning('cannot remember the route', error=str(error))
        return response

    def load(self, path: str) -> tuple[bool, str]:
        """Read, process, publish and send a route; (ok, message)."""
        cfg = self.config.value
        try:
            full = resolve_inside(cfg.sites_dir, path)
            raw = read_route(full)
        except (PathError, RouteError) as error:
            self.log.error('route not loaded', path=path, error=str(error))
            return False, str(error)
        if not 2 <= len(raw) <= MAX_POINTS:
            return False, f'{path}: a route needs 2 to {MAX_POINTS} points, got {len(raw)}'
        route, deviation = process(raw, cfg.processing) if cfg.process else (raw, 0.0)
        if deviation > cfg.processing.max_deviation:
            self.log.warning('smoothing moved the route', deviation_m=deviation)
        self.raw, self.route, self.source = raw, route, full
        self.publish_paths()
        self.waypoints_pub.publish(self.waypoint_msg())
        stops = sum(1 for p in route if p.is_stop)
        message = f'{os.path.basename(full)}: {len(raw)} -> {len(route)} points, {stops} stops'
        self.log.info('route loaded', path=full, points=len(route), stops=stops)
        return True, message

    def waypoint_msg(self) -> WaypointPath:
        """Return the route as a WaypointPath."""
        stamp = self.stamp()
        return waypoint_path(self.route, self.robot.frames.map, stamp)

    def publish_paths(self):
        """Publish the processed and the raw route for RViz."""
        stamp = self.stamp()
        frame = self.robot.frames.map
        self.path_pub.publish(nav_path(self.route, frame, stamp))
        self.raw_pub.publish(nav_path(self.raw, frame, stamp))


def main():
    """Run path_loader_node."""
    run(PathLoaderNode, NAME)


if __name__ == '__main__':
    main()
