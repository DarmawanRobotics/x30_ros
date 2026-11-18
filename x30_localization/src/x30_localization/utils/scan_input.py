"""Scan-to-map input of the node: /map and the registered cloud, throttled to rate_hz."""

from __future__ import annotations

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

from x30_localization.core.config import ScanMatchConfig
from x30_localization.core.sources.scan_match import (
    ScanBand,
    ScanMatchSource,
    match_config,
    planar_points,
)
from x30_localization.utils.conversions import occupancy
from x30_utils.math import Pose2D
from x30_utils.ros import LATCHED, SENSOR
from x30_utils.ros.pointcloud import CloudError, read_xyz
from x30_utils.ros.time import to_sec


class ScanInput:
    """Feeds scan-match measurements to node.on_scan_measurement(measurement, odom_base)."""

    def __init__(self, node, config: ScanMatchConfig, odom_frame: str, base_frame: str):
        self.node, self.config = node, config
        self.odom, self.base = odom_frame, base_frame
        self.source = ScanMatchSource(
            match_config(config.xy_window_m, config.yaw_window_deg, config.min_score),
            config.covariance_scale,
        )
        self.last = -1e9
        node.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        node.create_subscription(PointCloud2, config.cloud_topic, self.on_cloud, SENSOR)

    def on_map(self, msg: OccupancyGrid):
        """Use a new map."""
        grid = occupancy(msg, self.config.occupied_threshold)
        if isinstance(grid, str):
            self.node.log.warning('map not usable for scan matching', reason=grid)
            return
        self.source.set_map(*grid)
        self.node.log.info('scan matching on a new map', width=msg.info.width)

    def on_cloud(self, msg: PointCloud2):
        """Match a cloud at most rate_hz times per second, while localized."""
        stamp = to_sec(msg.header.stamp)
        if stamp - self.last < 1.0 / self.config.rate_hz or not self.node.is_localized():
            return
        self.last = stamp
        at = msg.header.stamp
        odom_body = self.node.tf.lookup(self.odom, msg.header.frame_id, at)
        odom_base = self.node.tf.lookup(self.odom, self.base, at)
        if odom_body is None or odom_base is None:
            self.node.log.debug('scan match skipped, no TF', error=self.node.tf.error)
            return
        try:
            points = read_xyz(msg)
        except CloudError as error:
            self.node.log.warning('cloud dropped', reason=str(error), throttle=10.0)
            return
        band = ScanBand(
            self.config.min_height_m, self.config.max_height_m, self.config.max_range_m
        )
        height = self.node.robot.base_height
        xy = planar_points(points, odom_body.matrix, odom_base.matrix, height, band)
        base = Pose2D.from_matrix(odom_base.matrix)
        prior = self.node.prior(base)
        if prior is None:
            return
        result = self.source.measure(xy, prior, stamp)
        if isinstance(result, str):
            self.node.log.debug('no scan match', reason=result)
            return
        self.node.on_scan_measurement(result, base)
