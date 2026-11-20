"""sim_robot_node: the X30 in software (driver, LiDARs, IMU, FAST-LIO); no Gazebo, no GPU."""

from __future__ import annotations

from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import BatteryState, Imu, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from x30_sim.core.config import SimConfig
from x30_sim.core.odometry import lio_transform
from x30_sim.core.scenario import Scenario, load_scenario
from x30_sim.core.simulation import Simulation
from x30_sim.core.world import World
from x30_sim.utils import conversions as conv
from x30_sim.utils import livox
from x30_sim.utils.files import scenario_file, world_file
from x30_utils.math import inverse, make_transform
from x30_utils.ros import COMMAND, EVENTS, X30Node, run
from x30_utils.ros.pointcloud import xyz_cloud
from x30_utils.ros.time import to_stamp

NAME = 'sim_robot_node'
LIO_ODOMETRY_TOPIC = '/Odometry'  # FAST-LIO's outputs
LIO_CLOUD_TOPIC = '/cloud_registered_body'
LIVOX_TOPIC = '/livox/lidar'  # the X30's merged LiDARs


class SimRobotNode(X30Node):
    """Kinematic X30 with four ray-cast Mid-360, a body IMU and drifting odometries."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.live = self.declare_config(SimConfig)
        self.robot = self.declare_robot().value
        path = world_file(self.live.world_file)
        world = World.from_yaml(path)
        scenario_path = scenario_file(self.live.scenario_file)
        scenario = load_scenario(scenario_path) if scenario_path else Scenario()
        self.sim = Simulation(self.live.value, self.robot, world, scenario)
        self.cfg = cfg = self.sim.config
        self.frames = self.robot.frames
        self.base_imu = make_transform(cfg.imu.xyz)
        # the map frame is where the odometry started, as on the robot: truth and obstacles in it
        self.start = world.start
        self._collision_sent: bool | None = None
        self._scans = 0
        self._create_interfaces(cfg)
        self._create_timers(cfg)
        self.log.info(
            'sim ready', world=path, scenario=scenario_path or 'none', seed=cfg.seed,
            walls=len(world.segments), pillars=len(world.circles), start=str(world.start),
        )  # fmt: skip

    def _create_interfaces(self, cfg: SimConfig):
        ns = cfg.driver_ns.strip('/')
        driver = (lambda name: f'/{ns}/{name}') if ns else (lambda name: name)
        self._livox_pub = self.create_publisher(livox.CustomMsgBytes, LIVOX_TOPIC, 5)
        self._lio_pub = self.create_publisher(Odometry, LIO_ODOMETRY_TOPIC, 20)
        self._cloud_pub = self.create_publisher(PointCloud2, LIO_CLOUD_TOPIC, 5)
        self._imu_pub = self.create_publisher(Imu, cfg.imu.topic, 50)
        self._odom_pub = self.create_publisher(Odometry, driver('odom'), 20)
        self._battery_pub = self.create_publisher(BatteryState, driver('battery'), 5)
        self._truth_pub = self.create_publisher(PoseStamped, 'sim/ground_truth', 10)
        self._collision_pub = self.latched(Bool, 'sim/collision')
        self.create_subscription(Twist, driver('cmd_vel'), self.on_cmd, COMMAND)
        self.create_subscription(PoseArray, 'sim/obstacles', self.on_obstacles, EVENTS)
        self.create_service(Trigger, 'sim/inspect', self.on_inspect)
        station = self.sim.station
        speed = lambda: float(np.hypot(self.sim.body.twist.vx, self.sim.body.twist.vy))  # noqa: E731
        for name, action in (('sit_down', lambda: station.sit(speed())),
                             ('stand_up', station.stand),
                             ('charge_start', lambda: station.start_charge(self.sim.body.pose)),
                             ('charge_stop', station.stop_charge)):  # fmt: skip
            self.create_service(Trigger, driver(name), self.posture_service(name, action))

    def _create_timers(self, cfg: SimConfig):
        self.every(1.0 / cfg.rate_hz, self.step)
        self.every(1.0 / cfg.odom_rate_hz, self.publish_leg_odometry)
        self.every(1.0 / cfg.lidar.rate_hz, self.publish_scan)
        if cfg.imu.rate_hz > 0.0:
            self.every(1.0 / cfg.imu.rate_hz, self.publish_imu)
        self.every(1.0, self.publish_battery)
        if cfg.camera.rate_hz > 0.0 and self.sim.camera.tags:
            self.every(1.0 / cfg.camera.rate_hz, self.publish_tags)
            if cfg.camera.publish_mount:
                camera = self.sim.camera.base_camera
                self.tf_out.send_static([(camera, self.frames.base, cfg.camera.frame)])

    # ------------------------------------------------------------------ inputs

    def on_cmd(self, msg: Twist):
        """Take a velocity command like x30_driver does."""
        self.sim.command(self.now(), conv.twist_from_msg(msg))

    def on_obstacles(self, msg: PoseArray):
        """Replace the dynamic obstacles (people, boxes) with circles at these map positions."""
        xy = self.start.apply(conv.obstacle_positions(msg))  # into the world
        self.sim.set_obstacles(xy, self.live.obstacle_radius, self.live.obstacle_height)
        self.log.info('dynamic obstacles set', count=len(xy))

    def on_inspect(self, _request, response):
        """Answer a stop-point inspection: always clear in the simulator."""
        pose = self.in_map(self.sim.body.pose)
        response.success = True
        response.message = f'clear (sim at {pose.x:.2f} {pose.y:.2f})'
        return response

    def posture_service(self, name: str, action):
        """Return a Trigger callback running a posture or charge action (sit_down, ...)."""

        def serve(_request, response):
            response.success, response.message = action()
            self.log.info('driver service', service=name, ok=response.success,
                          detail=response.message)  # fmt: skip
            return response

        return serve

    def in_map(self, pose):
        """Return a world pose in the map frame: the frame of the robot's start."""
        return self.start.inverse() @ pose

    # ------------------------------------------------------------------ simulation

    def step(self):
        """Advance the simulation by the time that really passed."""
        now = self.now()
        self.sim.step(now)
        truth = self.in_map(self.sim.body.pose)
        self._truth_pub.publish(conv.pose_stamped(now, self.frames.map, truth))
        collided = self.sim.body.collided
        if collided != self._collision_sent:
            self._collision_pub.publish(Bool(data=collided))
            self._collision_sent = collided
            if collided:
                self.log.error('collision: the body stopped', pose=str(self.sim.body.pose))

    def publish_scan(self):
        """Publish /livox/lidar, and FAST-LIO's odometry, TF and cloud of that scan."""
        now = self.now()
        scan = self.sim.scan(now)
        if scan is None:
            return
        stamp = to_stamp(scan.stamp)
        self._scans += 1
        data = livox.encode(scan.points, stamp, self.cfg.lidar.frame, self._scans)
        self._livox_pub.publish(livox.CustomMsgBytes(data))
        odom_base = self.sim.lio(scan.stamp, now)
        if odom_base is None:
            return
        lio = lio_transform(odom_base.to_matrix(), self.base_imu)
        parent, body = self.cfg.lio.odom_frame, self.cfg.lio.body_frame
        self._lio_pub.publish(conv.odometry_msg(scan.stamp, parent, body, lio))
        self.tf_out.send(lio, parent, body, stamp)
        points = scan.points[np.linalg.norm(scan.points, axis=1) > self.cfg.lio.blind]
        points = points - np.asarray(self.cfg.imu.xyz, dtype=np.float32)  # base -> IMU: offset
        self._cloud_pub.publish(xyz_cloud(conv.header(scan.stamp, body), points))

    def publish_leg_odometry(self):
        """Publish the driver's odom (leg odometry) and, with publish_tf, odom -> base_link."""
        sample = self.sim.leg_odometry(self.now())
        if sample is None:
            return
        f = self.frames
        msg = conv.leg_odometry_msg(sample.stamp, f.odom, f.base, sample.odom_base, sample.twist)
        self._odom_pub.publish(msg)
        if self.cfg.publish_tf:
            self.tf_out.send(sample.odom_base.to_matrix(), f.odom, f.base, msg.header.stamp)

    def publish_imu(self):
        """Publish the body IMU (m/s^2, rad/s) in imu_link."""
        now = self.now()
        sample = self.sim.imu(now)
        if sample is not None:
            self._imu_pub.publish(conv.imu_msg(now, self.cfg.imu.frame, *sample))

    def publish_battery(self):
        """Publish the battery once a second (no charging status unless report_charging)."""
        battery = self.cfg.battery
        msg = conv.battery_msg(
            self.now(), self.sim.battery, self.sim.station.charging, battery.report_charging
        )
        self._battery_pub.publish(msg)

    def publish_tags(self):
        """Publish the tags in view as TF camera -> tag, like the camera's tag detector."""
        now = self.now()
        stamp = to_stamp(now)
        camera_base = inverse(self.sim.camera.base_camera)
        for frame, base_tag in self.sim.tag_detections(now).items():
            self.tf_out.send(camera_base @ base_tag, self.cfg.camera.frame, frame, stamp)


def main():
    """Run sim_robot_node."""
    run(SimRobotNode, NAME)


if __name__ == '__main__':
    main()
