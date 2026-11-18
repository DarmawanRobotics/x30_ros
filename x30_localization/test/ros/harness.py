"""A fake X30 for in-process tests: URDF, FAST-LIO, a tilted IMU and a tag detector."""

from __future__ import annotations

import itertools
import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import Imu

from x30_localization.localization_node import LocalizationNode
from x30_msgs.msg import LocalizationStatus
from x30_utils.math import Pose2D, compose, inverse, make_transform, quat_from_rpy
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, delete_params, wait_for

BASE_IMU = make_transform((0.13045, -0.001, 0.0149))  # the X30's body IMU
BASE_CAM = make_transform((0.25, 0.0, 0.1), quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2))
START_TILT = (math.radians(2.0), math.radians(-1.5))  # base roll, pitch at start
BASE_HEIGHT = 0.48
TAG_IN_MAP = make_transform((2.5, 0.4, 0.5), quat_from_rpy(-math.pi / 2, 0.0, math.pi / 2))
NAMES = itertools.count()


class FakeRobot(X30Node):
    """Publishes what the X30, FAST-LIO and the detector would; the robot moves in the map."""

    def __init__(self, ns: str, imu_topic: str):
        super().__init__('fake_x30', param_ns=f'{ns}/fake_x30')
        self.tf_out.send_static(
            [
                (BASE_IMU, 'base_link', 'imu_link'),
                (BASE_CAM, 'base_link', 'camera_color_optical_frame'),
            ]
        )
        self.imu = self.create_publisher(Imu, imu_topic, 50)
        self.lock = threading.Lock()
        tilt = make_transform(quat=quat_from_rpy(*START_TILT, 0.0))
        self.world_start = compose(make_transform((0.0, 0.0, BASE_HEIGHT)), tilt)  # base at start
        self.robot = Pose2D()  # true planar pose of base in the (level) world = map
        self.tag_visible = False
        self.lio_lag = 0.0  # s: FAST-LIO's TF stamped this much behind the camera's
        self.every(0.02, self.publish, steady=True)

    def world_base(self) -> np.ndarray:
        """Return the true 3D base pose: planar motion on the floor, the start tilt kept."""
        with self.lock:
            return compose(self.robot.to_matrix(), self.world_start)

    def publish(self):
        stamp = self.stamp()
        world_base = self.world_base()
        init_body = inverse(self.world_start @ BASE_IMU) @ world_base @ BASE_IMU
        lio_stamp = stamp - rospy.Duration.from_sec(self.lio_lag)  # the robot stands still then
        self.tf_out.send(init_body, 'camera_init', 'body', lio_stamp)
        if self.tag_visible:
            cam_tag = inverse(world_base @ BASE_CAM) @ TAG_IN_MAP
            self.tf_out.send(cam_tag, 'camera_color_optical_frame', 'base_map', stamp)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        up = (world_base @ BASE_IMU)[:3, :3].T @ np.array([0.0, 0.0, 9.81])  # m/s^2
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            float(v) for v in up
        )
        self.imu.publish(imu)


class Harness:
    """The node under test, the fake robot and a probe, in this process."""

    def __init__(self, overrides: dict):
        self.ns = f'/localization_test_{next(NAMES)}'
        imu_topic = f'{self.ns}/imu'
        self.node = LocalizationNode(
            param_ns=f'{self.ns}/localization_node',
            overrides={'leveling.imu_topic': imu_topic, **overrides},
        )
        self.robot = FakeRobot(self.ns, imu_topic)
        self.probe = X30Node('probe', param_ns=f'{self.ns}/probe')
        self.status = Collector(LocalizationStatus, 'localization/status')

    def call(self, srv_type, name: str, **request):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, srv_type)(**request)

    def bridge_up(self, timeout: float = 10.0) -> bool:
        return wait_for(lambda: self.node.bridge is not None, timeout)

    def state(self) -> int:
        last = self.status.last
        return -1 if last is None else last.state

    def lookup(self, target: str, source: str) -> np.ndarray | None:
        result = self.probe.tf.lookup(target, source)
        return None if result is None else result.matrix

    def close(self):
        self.status.close()
        for node in (self.node, self.robot, self.probe):
            node.run_shutdown_hooks()
            node.destroy_node()
        delete_params(self.ns)
