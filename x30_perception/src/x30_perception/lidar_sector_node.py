"""lidar_sector_node: LiDAR (and depth camera) points -> front and 360 sector scans."""

from __future__ import annotations

import time

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from x30_msgs.msg import SectorScan
from x30_perception.core.config import PerceptionConfig
from x30_perception.core.depth import DepthCloud, DepthFilter
from x30_perception.core.filtering import tilt_rotation
from x30_perception.core.livox import TYPES, LivoxError, parse_custom_msg
from x30_perception.core.pipeline import Frame, ScanResult, SectorPipeline
from x30_perception.utils.conversions import sector_scan
from x30_perception.utils.markers import scan_markers
from x30_utils.math import Pose2D
from x30_utils.ros import COMMAND, SENSOR, X30Node, run
from x30_utils.ros.pointcloud import CloudError, read_xyz
from x30_utils.ros.time import to_sec, to_stamp

NAME = 'lidar_sector_node'
WATCHDOG_SEC = 0.1
FLOOR_LOG_SEC = 60.0  # how often the fitted floor (the camera mount as seen) is logged


class LidarSectorNode(X30Node):
    """Bins LiDAR points into sectors for avoidance (front) and the protective field (360)."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(PerceptionConfig).value
        self.robot = self.declare_robot().value
        self.pipeline = SectorPipeline(self.config, self.robot)
        self.base = self.robot.frames.base
        self._last_ok = time.monotonic()
        self.tf.static(self.base, self.config.input.frame)  # start listening now
        self._front_pub = self.create_publisher(SectorScan, 'perception/sector_scan', COMMAND)
        self._safety_pub = (
            self.create_publisher(SectorScan, 'perception/safety_scan', COMMAND)
            if self.config.safety.enabled
            else None
        )
        self._markers_pub = (
            self.create_publisher(MarkerArray, 'perception/obstacles', 1)
            if self.config.publish_markers
            else None
        )
        self._subscribe(self.config.input)
        self.depth = DepthFilter(self.config, self.robot) if self.config.depth.enabled else None
        self._depth_last = None  # (newest DepthCloud, monotonic time its cloud arrived)
        self._floor_logged = -FLOOR_LOG_SEC
        if self.depth is not None:
            self.tf.static(self.base, self.config.depth.frame)
            self.create_subscription(PointCloud2, self.config.depth.topic, self.on_depth, SENSOR)
        self.every(WATCHDOG_SEC, self._watchdog, steady=True)
        self.log.info(
            'lidar sectors ready',
            input=self.config.input.type,
            front=self.pipeline.front.num_sectors,
            safety=self.pipeline.safety.num_sectors,
            base=self.base,
            depth=self.config.depth.topic if self.depth is not None else 'off',
        )

    def _subscribe(self, cfg):
        if cfg.type == 'pointcloud2':
            self.create_subscription(PointCloud2, cfg.pointcloud_topic, self.on_cloud, SENSOR)
            return
        # the serialized bytes: no Livox driver needed here, the publisher's type is checked
        self.create_subscription(TYPES, cfg.livox_topic, self.on_livox, SENSOR, raw=True)

    def on_livox(self, data: bytes):
        """Process one serialized Livox CustomMsg (raw subscription callback)."""
        try:
            scan = parse_custom_msg(data)
        except LivoxError as error:
            self.log.error('Livox scan dropped', reason=str(error), throttle=2.0)
            return
        self._guarded(scan.frame_id, scan.stamp, scan.xyz, scan.tags)

    def on_cloud(self, msg: PointCloud2):
        """Process one PointCloud2."""
        try:
            xyz = read_xyz(msg)
        except CloudError as error:
            self.log.error('cloud dropped', reason=str(error), throttle=2.0)
            return
        self._guarded(msg.header.frame_id, to_sec(msg.header.stamp), xyz, None)

    def on_depth(self, msg: PointCloud2):
        """Filter one depth camera cloud into obstacles: the next LiDAR scan merges the newest."""
        received = time.monotonic()
        try:
            cloud = self._filter_depth(msg, received)
        except Exception:
            self.log.exception('depth cloud processing failed', throttle=2.0)
            self.depth.reset()
            cloud = DepthCloud.unusable('internal error')
        self._depth_last = (cloud, received)

    def _depth_now(self) -> DepthCloud | None:
        """Return the newest depth cloud (unusable when too old); None: no depth camera."""
        if self.depth is None:
            return None
        if self._depth_last is None:
            return DepthCloud.unusable('no cloud yet')
        cloud, received = self._depth_last
        age = time.monotonic() - received
        if age > self.config.depth.timeout_sec:
            return DepthCloud.unusable(f'no cloud for {age:.1f} s')
        return cloud

    def _filter_depth(self, msg: PointCloud2, received: float) -> DepthCloud:
        frame_id = msg.header.frame_id or self.config.depth.frame
        try:
            xyz = read_xyz(msg)
        except CloudError as error:
            self.depth.reset()
            return DepthCloud.unusable(f'malformed cloud: {error}')
        mount = self.tf.static(self.base, frame_id)
        if mount is None:
            self.depth.reset()
            return DepthCloud.unusable(f'no TF {self.base} <- {frame_id}')
        tilt = None
        if self.config.filter.level_source == 'tf':
            odom_base = self._odom_base(to_sec(msg.header.stamp))
            tilt = None if odom_base is None else tilt_rotation(odom_base)
        cloud = self.depth.process(xyz, mount, tilt, received)  # confirmed in arrival time
        self._log_floor(cloud)
        return cloud

    def _log_floor(self, cloud: DepthCloud):
        """Log the floor the camera sees now and then: a steady offset is a mount to correct."""
        now = time.monotonic()
        quiet = now - self._floor_logged < FLOOR_LOG_SEC
        if quiet or cloud.reason or not self.config.depth.floor_fit:
            return
        self._floor_logged = now
        floor = cloud.floor
        if not floor.fitted:
            self.log.info('depth camera: nominal floor kept', reason=floor.note)
            return
        pitch, roll = floor.mount_error_deg
        self.log.info(
            'depth camera floor fitted: the mount looks this far off its TF',
            pitch_deg=round(pitch, 1),
            roll_deg=round(roll, 1),
            floor_m=round(floor.c, 3),
        )

    def _guarded(self, frame_id: str, stamp: float, xyz, tags):
        try:
            self._process(frame_id or self.config.input.frame, stamp, xyz, tags)
        except Exception:
            self.log.exception('cloud processing failed, scans BLOCKED', throttle=2.0)
            self._publish(stamp, self.pipeline.blocked('internal error'))

    def _process(self, frame_id: str, stamp: float, xyz, tags):
        mount = self.tf.static(self.base, frame_id)
        if mount is None:
            self.log.warning(
                'waiting for the sensor mount TF', source=frame_id, target=self.base,
                error=self.tf.error, throttle=2.0,
            )  # fmt: skip
            self._publish(stamp, self.pipeline.blocked(f'no TF {self.base} <- {frame_id}'))
            return
        odom_base = self._odom_base(stamp)
        tilt = None if odom_base is None else tilt_rotation(odom_base)
        pose = None if odom_base is None else Pose2D.from_matrix(odom_base)
        depth = self._depth_now()
        result = self.pipeline.process(Frame(xyz, mount, stamp, tags, tilt, pose), depth)
        if depth is not None and depth.reason and not result.blocked_reason:
            self.log.warning(
                'depth camera not merged: scans from the LiDAR alone',
                reason=depth.reason,
                throttle=5.0,
            )
        if result.blocked_reason:
            self.log.warning('scans BLOCKED', reason=result.blocked_reason, throttle=2.0)
        else:
            self._last_ok = time.monotonic()
        self._publish(stamp, result)

    def _odom_base(self, stamp: float):
        """Return odom <- base at the cloud stamp (or the latest fresh one), or None."""
        odom = self.robot.frames.odom
        result = self.tf.lookup(odom, self.base, to_stamp(stamp))
        if result is None:
            result = self.tf.fresh(odom, self.base, self.config.filter.tilt_max_age)
        return None if result is None else result.matrix

    def _watchdog(self):
        """Publish BLOCKED scans while no usable cloud arrives (sensor or driver down)."""
        silent = time.monotonic() - self._last_ok
        if silent > self.config.input.timeout_sec:
            self.log.warning('no usable LiDAR data, scans BLOCKED', silent=silent, throttle=5.0)
            self._publish(self.now(), self.pipeline.blocked(f'no usable cloud for {silent:.1f} s'))

    def _publish(self, stamp: float, result: ScanResult):
        header = Header(frame_id=self.base, stamp=to_stamp(stamp))
        self._front_pub.publish(sector_scan(header, self.pipeline.front, result.front))
        if self._safety_pub is not None:
            self._safety_pub.publish(sector_scan(header, self.pipeline.safety, result.safety))
        if self._markers_pub is not None:
            marker_header = Header(frame_id=self.base)  # stamp 0: RViz uses the latest TF
            self._markers_pub.publish(
                scan_markers(marker_header, self.pipeline.front, self.pipeline.safety, result)
            )


def main():
    """Run lidar_sector_node."""
    run(LidarSectorNode, NAME)


if __name__ == '__main__':
    main()
