"""grid_mapper_node: the 2D occupancy grid of a site from the odometry's deskewed scans."""

from __future__ import annotations

from collections import deque

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from x30_mapping.core.config import MapperConfig
from x30_mapping.core.grid import LogOddsGrid
from x30_mapping.core.integrator import Outcome, Scan, ScanIntegrator
from x30_mapping.core.map_io import crop_known, save_map
from x30_mapping.core.session import MappingSession
from x30_mapping.core.site_store import SiteStore
from x30_mapping.utils.conversions import log_odds_msg, mapping_status
from x30_msgs.msg import MappingStatus
from x30_msgs.srv import LoadSite, StartMapping
from x30_utils.io import PathError
from x30_utils.ros import SENSOR, X30Node, run
from x30_utils.ros.pointcloud import CloudError, read_xyz
from x30_utils.ros.time import to_sec, to_stamp

NAME = 'grid_mapper_node'


class GridMapperNode(X30Node):
    """Integrate scans into the grid of one site while mapping; save and load it by site."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(MapperConfig).value
        self.robot = self.declare_robot().value
        self.frames = self.robot.frames
        self.store = SiteStore(self.config.sites_dir)
        self.session = MappingSession()
        self.integrator = ScanIntegrator(self.config, self.robot, self._new_grid())
        self._clouds: deque = deque(maxlen=4)
        self._cloud_sub = None
        self._dirty = False
        self._message = 'idle'
        self._grid_pub = self.latched(OccupancyGrid, 'mapping/grid')
        self._status_pub = self.latched(MappingStatus, 'mapping/status')
        self.create_service(StartMapping, 'grid_mapper/start', self._on_start)
        self.create_service(Trigger, 'grid_mapper/stop', self._on_stop)
        self.create_service(LoadSite, 'grid_mapper/save', self._on_save)
        self.create_service(LoadSite, 'grid_mapper/load', self._on_load)
        self.every(1.0 / self.config.rate_hz, self._process_tick)
        self.every(1.0 / self.config.publish_hz, self._publish_tick)
        self._publish_status()
        self.log.info('grid mapper ready', sites_dir=self.store.root, map=self.frames.map)

    @property
    def grid(self) -> LogOddsGrid:
        """The grid of the current session."""
        return self.integrator.grid

    def _new_grid(self) -> LogOddsGrid:
        return LogOddsGrid(self.config.grid)

    def _on_cloud(self, msg: PointCloud2):
        if self._cloud_sub is not None:  # not a delivery that raced the unsubscribe
            self._clouds.append(msg)

    def _subscribe(self, on: bool):
        if on and self._cloud_sub is None:
            self._cloud_sub = self.create_subscription(
                PointCloud2, self.config.cloud_topic, self._on_cloud, SENSOR
            )
        elif not on and self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)  # stops the deliveries
            self._cloud_sub = None
            self._clouds.clear()

    def _scan(self, msg: PointCloud2) -> Scan | None:
        """Return the scan with its transforms, or None while TF has no answer yet."""
        frames, stamp, sensor = self.frames, msg.header.stamp, msg.header.frame_id
        base_sensor = self.tf.static(frames.base, sensor)
        map_odom = self.tf.lookup(frames.map, frames.odom, stamp)
        odom_sensor = self.tf.lookup(frames.odom, sensor, stamp)
        if base_sensor is None or map_odom is None or odom_sensor is None:
            return None
        before = to_stamp(to_sec(stamp) - self.config.gate.correction_window_sec)
        window = (
            self.tf.lookup(frames.map, frames.odom, before),
            self.tf.lookup(frames.map, frames.odom),  # the newest map->odom
        )
        return Scan(
            read_xyz(msg),
            to_sec(stamp),
            base_sensor,
            map_odom.matrix,
            odom_sensor.matrix,
            tuple(w.matrix for w in window if w is not None),
        )

    def _process_tick(self):
        """Integrate the newest queued scan whose transforms are available."""
        if not self.session.active or not self._clouds:
            return
        for msg in reversed(list(self._clouds)):
            try:
                scan = self._scan(msg)
            except CloudError as error:
                self._clouds.remove(msg)
                self.log.error('scan dropped', reason=str(error), throttle=5.0)
                continue
            if scan is None:
                continue
            self._clouds.clear()
            self._integrate(scan)
            return
        now = self.now()
        max_age = self.config.max_cloud_age_sec
        old = [m for m in self._clouds if now - to_sec(m.header.stamp) > max_age]
        for msg in old:
            self._clouds.remove(msg)
        if old:
            self.log.warning(
                'scans dropped: no TF for them (odometry, localization running?)',
                error=self.tf.error,
                throttle=10.0,
            )

    def _integrate(self, scan: Scan):
        result = self.integrator.process(scan)
        if result.outcome is Outcome.INTEGRATED:
            self._dirty = True
            self._message = f'mapping {self.session.site}, {self.grid.scans} scans'
            if self.grid.capped:
                self._message += ' (grid at max_size_m: far hits dropped)'
            return
        if result.rejected:
            self._message = f'scan rejected: {result.message}'
            self._dirty = True  # status shows the rejection
            self.log.warning('scan rejected', reason=result.message, throttle=5.0)

    def _publish_tick(self):
        if self.session.active and self._dirty:
            self._dirty = False
            self._grid_pub.publish(self._grid_msg())
            self._publish_status()

    def _grid_msg(self) -> OccupancyGrid:
        return log_odds_msg(self.grid, self.frames.map, self.stamp())

    def _publish_status(self):
        self._status_pub.publish(
            mapping_status(
                self.stamp(),
                self.session.active,
                self.session.site,
                self.grid,
                self.integrator.rejected,
                self._message,
            )
        )

    def _done(self, response, ok: bool, message: str):
        response.success, response.message = ok, message
        (self.log.info if ok else self.log.error)(message)
        self._publish_status()
        return response

    def _on_start(self, request, response):
        """Service: map a site; reset starts its grid from scratch, else the grid continues."""
        site = request.site.strip()
        try:
            self.store.layout(site)
        except PathError as error:
            return self._done(response, False, str(error))
        error = self.session.start(site, request.reset)
        if error:
            return self._done(response, False, error)
        self.integrator.reset(self._new_grid() if request.reset else self.grid)
        self._subscribe(True)
        self._dirty = True
        how = 'from scratch' if request.reset else 'continuing its map'
        self._message = f'mapping {site} {how}'
        return self._done(response, True, self._message)

    def _on_stop(self, request, response):
        """Service: stop integrating scans; the grid stays with its site for saving."""
        self.session.stop()
        self._subscribe(False)
        self._message = f'stopped {self.session.site or "-"} after {self.grid.scans} scans'
        self._grid_pub.publish(self._grid_msg())
        return self._done(response, True, self._message)

    def _on_save(self, request, response):
        """Service: write the grid into its own site (refused for any other site)."""
        site = request.site.strip()
        error = self.session.check_save(site)
        if error:
            return self._done(response, False, error)
        directory = self.store.layout(site).root
        if not self.store.exists(site):
            return self._done(response, False, f'site directory {directory} does not exist')
        grid = crop_known(self.grid.to_grid_map(), self.config.save_margin_m)
        if not (grid.data >= 0).any():
            return self._done(response, False, 'nothing mapped yet')
        try:
            path = save_map(directory, grid)
        except OSError as error:
            return self._done(response, False, f'cannot save in {directory}: {error}')
        area = self.grid.known_area()
        return self._done(
            response, True, f'saved {path}: {grid.width}x{grid.height}, {area:.0f} m2'
        )

    def _on_load(self, request, response):
        """Service: take a site's saved map as the grid to continue."""
        site = request.site.strip()
        if self.session.active:
            return self._done(response, False, f'mapping {self.session.site}: stop first')
        try:
            grid, problems = self.store.load_map(site)
        except PathError as error:
            return self._done(response, False, str(error))
        if grid is None:
            return self._done(response, False, f'{site} has no readable map {problems}')
        prior = self._new_grid()
        try:
            prior.load_prior(grid)
        except ValueError as error:
            return self._done(response, False, f'{site}: {error}')
        self.session.load(site)
        self.integrator.reset(prior)
        self._message = f'{site} loaded as the prior ({grid.width}x{grid.height})'
        return self._done(response, True, self._message)


def main():
    """Run grid_mapper_node."""
    run(GridMapperNode, NAME)


if __name__ == '__main__':
    main()
