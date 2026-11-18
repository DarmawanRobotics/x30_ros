"""localization_node: the REP-105 bridge and map->odom from an EKF fed by tags and operators."""

from __future__ import annotations

import math
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from x30_localization.core.bridge import Bridge, make_bridge
from x30_localization.core.config import LocalizationConfig
from x30_localization.core.estimator import MapOdomEstimator
from x30_localization.core.leveling import Leveler, StillLimits
from x30_localization.core.localizer import Localizer
from x30_localization.core.sources import PoseEstimateSource
from x30_localization.core.tags import describe
from x30_localization.utils.conversions import (
    imu_vectors,
    initial_pose,
    pose_msg,
    status_msg,
)
from x30_localization.utils.markers import tag_markers
from x30_localization.utils.scan_input import ScanInput
from x30_localization.utils.tag_input import TagInput
from x30_localization.utils.tag_services import TagServices
from x30_msgs.msg import LocalizationStatus
from x30_utils.io import expand
from x30_utils.math import Pose2D
from x30_utils.ros import SENSOR, X30Node, run
from x30_utils.ros.time import to_stamp

NAME = 'localization_node'


class LocalizationNode(X30Node):
    """Publishes the bridge and map->odom; tags, pose estimates and scans correct it."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(LocalizationConfig)
        self.robot = self.declare_robot()
        cfg, frames = self.config.value, self.robot.frames
        self.map, self.odom, self.base = frames.map, frames.odom, frames.base
        self.lock = threading.RLock()
        self.localizer = Localizer(MapOdomEstimator(cfg.estimator), cfg.tag)
        self.config.on_change(self.on_config)
        lv = cfg.leveling
        limits = StillLimits(lv.window_sec, math.radians(lv.max_rate_deg_s),
                             lv.max_accel_spread, math.radians(lv.max_tilt_deg))  # fmt: skip
        self.leveler = Leveler(limits, lv.timeout_sec, lv.enabled)
        self.bridge: Bridge | None = None
        self.relevel = True  # (re)compute the bridge: at start and after an odometry restart
        self.status_pub = self.latched(LocalizationStatus, 'localization/status')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'localization/pose', 10)
        self.markers_pub = self.latched(MarkerArray, 'localization/tag_markers')
        self.tag_input = TagInput(self, cfg.tag.camera_frame)
        self.every(1.0 / cfg.publish_hz, self.tick)
        self.every(1.0 / cfg.tag.update_hz, self.tag_input.poll)
        self.every(1.0, self.publish_status)
        if cfg.pose_hz > 0.0:
            self.every(1.0 / cfg.pose_hz, self.publish_pose)
        self.create_subscription(Empty, 'odometry/restarted', self.on_restart, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_pose_estimate, 10
        )
        if lv.enabled:
            self.create_subscription(Imu, lv.imu_topic, self.on_imu, SENSOR)
        if cfg.scan_match.enabled:
            ScanInput(self, cfg.scan_match, self.odom, self.base)
        for name, callback in (('start', self.srv_start), ('reset', self.srv_reset),
                               ('assume_origin', self.srv_assume_origin)):  # fmt: skip
            self.create_service(Trigger, f'localization/{name}', callback)
        self.tag_files = TagServices(self)
        self.tag_files.load(expand(cfg.tags_file))
        self.publish_status()

    # ---------------------------------------------------------------- odometry and TF

    def odom_base(self, stamp=None) -> Pose2D | None:
        """Return odom->base through the bridge (latest, or at stamp), or None."""
        if self.bridge is None:
            return None
        cfg = self.config.value
        lio = self.tf.lookup(cfg.lio_odom_frame, cfg.lio_body_frame, stamp)
        return None if lio is None else Pose2D.from_matrix(self.bridge.odom_base(lio.matrix))

    def odom_base_matrix(self, stamp):
        """Return the 4x4 odom->base at stamp (full 3D, for recording), or None."""
        cfg = self.config.value
        lio = self.tf.lookup(cfg.lio_odom_frame, cfg.lio_body_frame, stamp)
        return None if lio is None or self.bridge is None else self.bridge.odom_base(lio.matrix)

    def tick(self):
        """Bring the bridge up, follow odometry, publish map->odom, end recordings."""
        if self.relevel:
            self.try_bridge()
        if self.bridge is None:
            return
        odom_base = self.odom_base()
        with self.lock:
            now = self.now()
            samples = (
                self.localizer.tick(now)
                if odom_base is None
                else self.localizer.track(odom_base, now)
            )
            self.send(samples)
        self.tag_files.tick(now)

    def send(self, samples):
        """Broadcast map->odom samples (old value just before a change, then the new one)."""
        for pose, stamp in samples:
            self.tf_out.send(pose.to_matrix(), self.map, self.odom, to_stamp(stamp))

    def try_bridge(self):
        """Publish the static bridge once the URDF and the IMU allow it (leveled odom)."""
        cfg = self.config.value
        base_imu = self.tf.static(self.base, cfg.imu_frame)
        lio = self.tf.lookup(cfg.lio_odom_frame, cfg.lio_body_frame)
        if base_imu is None or lio is None:
            self.log.info(
                'waiting for the URDF and the odometry', error=self.tf.error, throttle=10.0
            )
            return
        level, decided, reason = self.leveler.level(base_imu, lio.matrix, self.now())
        if not decided:
            self.log.info('waiting to level odom', reason=reason, throttle=5.0)
            return
        if reason and self.leveler.enabled:
            self.log.warning('odom not gravity-aligned', reason=reason)
        first = self.bridge is None
        self.bridge, self.relevel = make_bridge(base_imu, level), False
        self.tf_out.send_static(
            [
                (self.bridge.odom_init, self.odom, cfg.lio_odom_frame),
                (self.bridge.body_base, cfg.lio_body_frame, self.base),
            ]
        )
        self.log.event(
            'bridge_up', leveled=self.bridge.leveled, tilt=describe(self.bridge.odom_init)
        )
        if cfg.assume_start_origin and first:  # never after an odometry restart
            with self.lock:
                self.send(self.localizer.assume_origin(self.now()))
            self.publish_status()

    def on_imu(self, msg: Imu):
        """Collect IMU samples for leveling (only needed until the bridge is up)."""
        if self.relevel:
            self.leveler.add(*imu_vectors(msg))

    def on_restart(self, _msg: Empty):
        """Forget map->odom and the level: the odometry restarted at a new origin."""
        with self.lock:
            self.send(self.localizer.reset(self.now(), 'odometry restarted: localize again'))
        self.leveler.restart()
        self.relevel = True  # the old bridge (and map->odom = identity) stay up meanwhile
        self.log.error('odometry restarted, map->odom invalid: show a tag or set a pose estimate')
        self.publish_status()

    # ---------------------------------------------------------------- sources

    def is_localized(self) -> bool:
        """Return True while map->odom is known."""
        return self.localizer.estimator.localized

    def prior(self, odom_base: Pose2D) -> Pose2D | None:
        """Return the believed map->base for an odometry pose, or None."""
        map_odom = self.localizer.estimator.map_odom
        return None if map_odom is None else map_odom @ odom_base

    def on_scan_measurement(self, measurement, odom_base: Pose2D):
        """Fuse a scan-to-map match through the gate."""
        with self.lock:
            if not self.is_localized():
                return
            result, samples = self.localizer.correct(measurement, odom_base, self.now())
            self.send(samples)
        if not result.accepted:
            self.log.info('scan match ' + result.describe(), throttle=10.0)

    def on_pose_estimate(self, msg: PoseWithCovarianceStamped):
        """RViz 2D Pose Estimate: the operator sets map->base (no gate)."""
        if msg.header.frame_id and msg.header.frame_id != self.map:
            self.log.error('pose estimate ignored', frame=msg.header.frame_id, expected=self.map)
            return
        odom_base = self.odom_base()
        if odom_base is None:
            self.log.error('pose estimate ignored: odometry or bridge not ready')
            return
        pose, cov, _ = initial_pose(msg)
        measurement = PoseEstimateSource().measure(pose, cov, self.now())
        if isinstance(measurement, str):
            self.log.error('pose estimate ignored', reason=measurement)
            return
        with self.lock:
            self.send(self.localizer.localize(measurement, odom_base, self.now()))
        self.log.event('pose_estimate', x=pose.x, y=pose.y, yaw=pose.yaw)
        self.publish_status()

    def on_config(self, _old, new: LocalizationConfig):
        """Apply live parameters."""
        with self.lock:
            self.localizer.configure(new.tag)
            self.localizer.estimator.config = new.estimator

    # ---------------------------------------------------------------- services

    def srv_start(self, _request, response):
        """Localize from a tag in view, explicitly (no gate: a far consistent tag relocalizes)."""
        response.success, response.message = self.tag_input.localize_now()
        (self.log.info if response.success else self.log.error)(response.message)
        self.publish_status()
        return response

    def srv_reset(self, _request, response):
        """Forget map->odom; tags in view localize again (auto_localize)."""
        with self.lock:
            was = self.localizer.estimator.source or 'none'
            self.send(self.localizer.reset(self.now(), 'reset: localize again'))
        response.success, response.message = True, f'localization reset (was {was})'
        self.log.event('reset', was=was)
        self.publish_status()
        return response

    def srv_assume_origin(self, _request, response):
        """Take the current odometry frame as the map (a new map starts here)."""
        if self.bridge is None or self.odom_base() is None:
            response.success = False
            response.message = 'frame bridge not ready (URDF, odometry or leveling)'
            return response
        with self.lock:
            self.send(self.localizer.assume_origin(self.now()))
        response.success, response.message = True, 'map->odom = identity: the map starts here'
        self.publish_status()
        return response

    # ---------------------------------------------------------------- outputs

    def publish_status(self):
        """Publish localization/status (latched; also once a second)."""
        with self.lock:
            status = self.localizer.status(self.now())
        self.status_pub.publish(status_msg(status, self.stamp(), self.map))

    def publish_pose(self):
        """Publish map->base with its covariance while localized."""
        with self.lock:
            est = self.localizer.estimator
            pose, cov = est.base_pose(), est.base_covariance()
        if pose is not None and self.bridge is not None:
            self.pose_pub.publish(pose_msg(pose, cov, self.stamp(), self.map))

    def publish_markers(self):
        """Publish the known tags (latched)."""
        tags = dict(self.localizer.tags)
        stamp = self.stamp()
        self.markers_pub.publish(tag_markers(tags, self.map, self.config.tag.marker_size, stamp))


def main():
    """Run the node (record_tags runs beside the timers that finish it)."""
    run(LocalizationNode, NAME)


if __name__ == '__main__':
    main()
