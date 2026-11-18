"""Watch the LiDAR-inertial odometry: when it can no longer be trusted, and how far it went."""

from __future__ import annotations

import math
import time

from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from x30_msgs.msg import OdometryStatus, TripStatus
from x30_odometry.core.config import MonitorConfig
from x30_odometry.core.monitor import EventKind, OdometryMonitor, OdometryState
from x30_odometry.core.restart import PoseSample
from x30_odometry.core.trip import TripMeter, TripTotals
from x30_utils.io import read_json, write_json_atomic
from x30_utils.io.paths import expand
from x30_utils.ros import EVENTS, SENSOR, X30Node, run
from x30_utils.ros.time import to_sec, to_stamp

NAME = 'odometry_monitor_node'


def robot_speed_limit(robot) -> float:
    """Return the fastest planar speed of the robot (m/s)."""
    return math.hypot(robot.limits.max_vx, robot.limits.max_vy)


class OdometryMonitorNode(X30Node):
    """odometry/status, odometry/restarted and odometry/trip from /Odometry."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(MonitorConfig)
        self.robot = self.declare_robot()
        self.monitor = OdometryMonitor(self.config.value, robot_speed_limit(self.robot.value))
        self.config.on_change(lambda _old, _new: self.reconfigure())
        self.robot.on_change(lambda _old, _new: self.reconfigure())
        self.last_state: OdometryState | None = None

        c = self.config
        self.status_pub = self.latched(OdometryStatus, 'odometry/status')
        self.restart_pub = self.create_publisher(Empty, 'odometry/restarted', EVENTS)
        self.create_subscription(Odometry, c.odom_topic, self.on_odometry, 20)
        if c.leg_odom_topic:
            self.create_subscription(Odometry, c.leg_odom_topic, self.on_leg_odometry, 20)
        if c.imu_topic:
            self.create_subscription(Imu, c.imu_topic, self.on_imu, SENSOR)
        self.every(c.status_period_sec, self.publish_status, steady=True)
        self.publish_status()

        self.trip: TripMeter | None = None
        self.last_pose_rx = 0.0
        if c.trip.enabled:
            self.trip = TripMeter(c.trip, time.time())
            self.trip_file = expand(c.trip.file)
            self.trip_saved = time.monotonic()
            self.load_trip()
            self.trip_pub = self.latched(TripStatus, 'odometry/trip')
            self.create_service(Trigger, 'odometry/trip_reset', self.on_trip_reset)
            self.every(c.trip.period_sec, self.publish_trip, steady=True)
            self.add_shutdown_hook(lambda: self.save_trip(force=True))
            self.publish_trip()
        self.log.info(
            'watching odometry',
            odom=c.odom_topic,
            leg=c.leg_odom_topic or 'off',
            imu=c.imu_topic or 'off',
        )

    def reconfigure(self):
        """Apply changed parameters to the monitor and the trip meter."""
        self.monitor.configure(self.config.value, robot_speed_limit(self.robot.value))
        if self.trip is not None:
            self.trip.configure(self.config.value.trip)

    def on_imu(self, msg: Imu):
        """Feed the stillness window (IMU axes = odometry body axes)."""
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.monitor.on_imu(time.monotonic(), (w.x, w.y, w.z), (a.x, a.y, a.z))

    def on_leg_odometry(self, msg: Odometry):
        """Feed the robot's own planar speed (twist in the base frame)."""
        v = msg.twist.twist.linear
        self.monitor.on_leg(time.monotonic(), v.x, v.y)

    def on_odometry(self, msg: Odometry):
        """Check every estimator pose for jumps and restarts."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = PoseSample(
            to_sec(msg.header.stamp), np.array([p.x, p.y, p.z]), (q.x, q.y, q.z, q.w)
        )
        if not (np.all(np.isfinite(pose.position)) and all(map(math.isfinite, pose.orientation))):
            self.log.warning('non-finite odometry pose ignored', throttle=5.0)
            return
        event = self.monitor.on_odometry(time.monotonic(), pose)
        if self.trip is not None:
            self.last_pose_rx = time.monotonic()
            if event is None:
                self.trip.add(pose.stamp, p.x, p.y)
            else:
                self.trip.gap()  # nothing counted across a jump or a restart
        if event is None:
            return
        if event.kind is EventKind.RESTART:
            self.log.error('odometry restarted: odom frame reset', reason=event.message)
            self.restart_pub.publish(Empty())
        else:
            self.log.error('odometry jumped', reason=event.message)
        self.publish_status()

    def publish_status(self):
        """Publish the current state (latched) and log transitions."""
        status = self.monitor.status(time.monotonic())
        msg = OdometryStatus(
            state=status.state.value,
            speed=float(status.speed),
            tilt=float(status.tilt),
            message=status.message,
        )
        msg.header.stamp = self.stamp()
        self.status_pub.publish(msg)
        if status.state is self.last_state:
            return
        if self.last_state is not None:
            if status.state is OdometryState.OK:
                self.log.info('odometry ok')
            else:
                self.log.warning(f'odometry {status.state.name.lower()}', reason=status.message)
        self.last_state = status.state

    # ---------------------------------------------------------------- trip

    def publish_trip(self):
        """Publish the trip (latched); keep the totals now and then."""
        trip = self.trip
        if time.monotonic() - self.last_pose_rx > self.config.trip.gap_sec:
            trip.gap()  # no odometry: standing, not at the last speed
        msg = TripStatus(
            distance=float(trip.distance),
            moving_sec=float(trip.moving_sec),
            speed=float(trip.speed),
            average=float(trip.average),
            max_speed=float(trip.max_speed),
        )
        msg.header.stamp = self.stamp()
        msg.since = to_stamp(trip.since)
        self.trip_pub.publish(msg)
        self.save_trip()

    def on_trip_reset(self, _request, response):
        """Start the trip again from zero."""
        trip = self.trip
        before = f'{trip.distance:.1f} m in {trip.moving_sec:.0f} s'
        trip.reset(time.time())
        self.save_trip(force=True)
        self.publish_trip()
        self.log.info('trip reset', before=before)
        response.success, response.message = True, f'trip reset (was {before})'
        return response

    def load_trip(self):
        """Continue from the kept totals, when there are any."""
        if not self.trip_file:
            return
        data, error = read_json(self.trip_file)
        totals = TripTotals.from_dict(data) if data is not None else None
        if error or (data is not None and totals is None):
            self.log.warning('trip file not usable, starting from zero', file=self.trip_file,
                             error=error or 'unexpected content')  # fmt: skip
            return
        if totals is not None:
            self.trip.restore(totals)
            self.log.info('trip continued', distance_m=round(totals.distance, 1),
                          file=self.trip_file)  # fmt: skip

    def save_trip(self, force: bool = False):
        """Write the totals when they changed, at most every trip.save_sec (force: now)."""
        if not self.trip_file or not (force or self.trip.changed):
            return
        now = time.monotonic()
        if not force and now - self.trip_saved < self.config.trip.save_sec:
            return
        self.trip_saved = now
        try:
            write_json_atomic(self.trip_file, self.trip.totals().to_dict(), backup=False)
        except OSError as error:
            self.log.warning(
                'trip not saved', file=self.trip_file, error=str(error), throttle=60.0
            )


def main():
    """Run the odometry monitor."""
    run(OdometryMonitorNode, NAME)


if __name__ == '__main__':
    main()
