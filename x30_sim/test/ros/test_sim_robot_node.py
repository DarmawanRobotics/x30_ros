"""In-process tests of sim_robot_node: outputs, frames, commands, collisions, latency, dropouts."""

from __future__ import annotations

import itertools
import math
from pathlib import Path
import time

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rospy
from sensor_msgs.msg import BatteryState, Imu, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from x30_msgs.msg import LocalizationStatus
from x30_perception.core.livox import parse_custom_msg
from x30_sim.sim_robot_node import SimRobotNode
from x30_utils.math import Pose2D
from x30_utils.ros import X30Node
from x30_utils.ros.pointcloud import read_xyz
from x30_utils.ros.time import to_sec
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

WORLD = str(Path(__file__).resolve().parents[2] / 'config' / 'worlds' / 'mall_loop.yaml')
IMU = np.array([0.13045, -0.001, 0.0149])
NAMES = itertools.count()


def same_scan(odometry: Odometry, clouds: list) -> bool:
    """Return True when a cloud has the stamp of the odometry (FAST-LIO: one per scan)."""
    stamp = to_sec(odometry.header.stamp)
    return any(abs(to_sec(c.header.stamp) - stamp) < 1e-9 for c in clouds)


class Rig:
    """The node under test (its driver topics in a namespace of its own) and collectors."""

    def __init__(self, **overrides):
        n = next(NAMES)
        self.ns = f'/sim_test_{n}'
        self.driver = f'{self.ns}/x30'
        values = {'world_file': WORLD, 'initial_pose_hz': 5.0, 'driver_ns': self.driver}
        self.node = SimRobotNode(param_ns=f'{self.ns}/node', overrides={**values, **overrides})
        self.probe = X30Node('probe', param_ns=f'{self.ns}/probe')
        self.livox = Collector(rospy.AnyMsg, '/livox/lidar')
        self.lio = Collector(Odometry, '/Odometry')
        self.cloud = Collector(PointCloud2, '/cloud_registered_body')
        self.imu = Collector(Imu, '/imu/data', 400)
        self.odom = Collector(Odometry, f'{self.driver}/odom')
        self.battery = Collector(BatteryState, f'{self.driver}/battery')
        self.truth = Collector(PoseStamped, 'sim/ground_truth', 200)
        self.collision = Collector(Bool, 'sim/collision')
        self.initial = Collector(PoseWithCovarianceStamped, '/initialpose')
        self.cmd_pub = rospy.Publisher(f'{self.driver}/cmd_vel', Twist, queue_size=10)
        self.obstacle_pub = rospy.Publisher('sim/obstacles', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher(
            'localization/status', LocalizationStatus, queue_size=1, latch=True
        )
        assert connected(self.cmd_pub, self.obstacle_pub, self.status_pub)

    def drive(self, vx: float, seconds: float, vy: float = 0.0, wz: float = 0.0):
        """Publish a command at 20 Hz for seconds."""
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_pub.publish(command)
            time.sleep(0.05)

    def place(self, pose: Pose2D):
        """Put the body at pose between two simulation steps, never inside one."""

        # a step running meanwhile integrates from the pose before and writes it back over
        # this one: run as one of the node's callbacks, one at a time with the steps
        def put():
            self.node.sim.body.pose = pose

        self.node.guard(put)()

    def call(self, name: str):
        """Call a Trigger service of the driver namespace (or an absolute one)."""
        name = name if name.startswith('/') else f'{self.driver}/{name}'
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, Trigger)()

    def close(self):
        """Unsubscribe, unadvertise, destroy the nodes."""
        for collector in (self.livox, self.lio, self.cloud, self.imu, self.odom, self.battery,
                          self.truth, self.collision, self.initial):  # fmt: skip
            collector.close()
        for publisher in (self.cmd_pub, self.obstacle_pub, self.status_pub):
            publisher.unregister()
        self.probe.destroy_node()
        self.node.destroy_node()
        delete_params(self.ns)


@pytest.fixture
def rig(ros):
    """A rig with the default world and parameters."""
    r = Rig()
    yield r
    r.close()


@pytest.fixture
def make(ros):
    """Rigs with parameter overrides."""
    made = []

    def factory(**overrides):
        made.append(Rig(**overrides))
        return made[-1]

    yield factory
    for r in made:
        r.close()


def test_publishes_what_the_stack_expects(rig):
    """The X30's /livox/lidar and /imu/data, FAST-LIO's outputs, the driver's, the truth."""
    for c in (rig.livox, rig.lio, rig.cloud, rig.imu, rig.odom, rig.battery, rig.truth):
        assert c.wait(1, 5.0), c.sub.name
    raw = rig.livox.last
    assert raw._connection_header['type'] == 'livox_ros_driver2/CustomMsg'
    scan = parse_custom_msg(raw._buff)
    assert scan.frame_id == 'livox_frame' and len(scan.xyz) > 1000
    lio, cloud, imu, odom = rig.lio.last, rig.cloud.last, rig.imu.last, rig.odom.last
    assert (lio.header.frame_id, lio.child_frame_id) == ('camera_init', 'body')
    assert wait_for(lambda: same_scan(rig.lio.last, rig.cloud.messages), 2.0)
    assert cloud.header.frame_id == 'body' and len(read_xyz(cloud)) >= 1000
    assert imu.header.frame_id == 'imu_link'
    assert imu.linear_acceleration.z == pytest.approx(9.81, abs=0.1)  # m/s^2
    assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_link')
    assert odom.pose.covariance[0] > 0.0 and odom.twist.covariance[35] > 0.0
    assert rig.battery.last.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    assert rig.truth.last.header.frame_id == 'map'
    assert rig.truth.last.pose.position.x == pytest.approx(-4.5, abs=1e-6)
    assert rig.collision.wait(1, 2.0) and rig.collision.last.data is False
    assert wait_for(lambda: rig.probe.tf.lookup('camera_init', 'body') is not None, 5.0)


def test_fast_lio_cloud_is_in_the_imu_frame_with_the_blind_radius(rig):
    """The south wall 1.5 m right of base_link; nothing within 0.7 m of base_link."""
    assert rig.cloud.wait(1, 5.0)
    points = read_xyz(rig.cloud.last)
    in_base = points + IMU
    above_floor = in_base[:, 2] > -0.3  # the floor is 0.48 m below base_link
    right = in_base[(np.abs(in_base[:, 0]) < 0.05) & (in_base[:, 1] < 0.0) & above_floor]
    assert len(right) > 0 and right[:, 1].max() == pytest.approx(-1.5, abs=0.06)
    assert np.linalg.norm(in_base, axis=1).min() > 0.7


def test_commands_move_the_robot_and_the_odometries_follow(rig):
    """cmd_vel drives the body forward; /Odometry and the leg odometry follow; stale: stop."""
    assert rig.truth.wait(1, 5.0)
    rig.drive(0.5, 1.5)
    moved = rig.truth.last.pose.position.x + 4.5
    assert moved > 0.2
    assert rig.lio.last.pose.pose.position.x == pytest.approx(moved, abs=0.1)
    assert rig.odom.last.pose.pose.position.x == pytest.approx(moved, abs=0.1)
    assert rig.odom.last.twist.twist.linear.x > 0.2
    time.sleep(1.5)  # timeout 0.5 s, then braking
    x = rig.truth.last.pose.position.x
    time.sleep(0.3)
    assert rig.truth.last.pose.position.x == pytest.approx(x, abs=1e-3)


def test_imu_reads_the_turn_and_gravity(rig):
    assert rig.imu.wait(10, 5.0)
    rig.drive(0.0, 2.0, wz=0.4)
    assert rig.imu.last.angular_velocity.z == pytest.approx(0.4, abs=0.02)
    assert rig.imu.last.linear_acceleration.z == pytest.approx(9.81, abs=0.1)


def test_initial_pose_only_while_not_localized(rig):
    """/initialpose at the true pose while localization is silent or NONE, not while LOCALIZED."""
    assert rig.initial.wait(1, 3.0)
    msg = rig.initial.last
    assert msg.header.frame_id == 'map' and msg.pose.pose.position.x == pytest.approx(-4.5)
    assert msg.pose.covariance[0] > 0.0
    rig.status_pub.publish(LocalizationStatus(state=LocalizationStatus.LOCALIZED))
    assert wait_for(lambda: rig.node.localization == LocalizationStatus.LOCALIZED, 3.0)
    time.sleep(0.3)  # one may still be on its way
    count = len(rig.initial.messages)
    time.sleep(0.6)
    assert len(rig.initial.messages) == count
    rig.status_pub.publish(LocalizationStatus(state=LocalizationStatus.NONE))
    assert rig.initial.wait(count + 1, 3.0)


def test_collision_latches_and_stops(rig):
    """A person dropped ahead: driving into them stops the body and latches sim/collision."""
    assert rig.collision.wait(1, 5.0)
    person = PoseArray(poses=[Pose()])
    person.poses[0].position.x, person.poses[0].position.y = -3.0, -3.0
    for _ in range(10):
        rig.obstacle_pub.publish(person)
        if wait_for(lambda: len(rig.node.sim.dynamic) == 1, 0.2):
            break
    rig.drive(0.6, 3.0)
    assert wait_for(lambda: rig.collision.last.data is True, 2.0)
    front = rig.truth.last.pose.position.x + rig.node.robot.footprint.front
    assert front == pytest.approx(-3.0 - 0.25, abs=0.06)  # the front edge stopped at the person
    rig.obstacle_pub.publish(PoseArray())
    time.sleep(0.3)
    assert rig.collision.last.data is True  # stays latched


def test_inspect_answers_clear(rig):
    response = rig.call('/sim/inspect')
    assert response.success and response.message.startswith('clear')


def test_driver_services_and_the_dock_tag_on_tf(rig):
    """The tag camera on TF; sit_down, charge_start, stand_up as x30_driver's (walking: no)."""
    station = rig.node.sim.station
    rig.place(Pose2D(-4.5, 0.0, math.pi))  # facing the dock tag on the west wall
    tf = rig.probe.tf
    assert wait_for(lambda: tf.lookup('base_link', 'charging_dock') is not None, 5.0)
    assert tf.lookup('base_link', 'charging_dock').matrix[0, 3] == pytest.approx(1.5, abs=0.02)
    camera = tf.static('base_link', 'camera_color_optical_frame')  # the detector's frame
    assert camera[:3, 3] == pytest.approx([0.5, 0.0, 0.1]) and camera[:3, 2] == pytest.approx(
        [1.0, 0.0, 0.0], abs=1e-9
    )  # optical z ahead
    in_view = tf.lookup('camera_color_optical_frame', 'charging_dock').matrix
    assert in_view[2, 3] == pytest.approx(1.0, abs=0.02)  # 1.0 m ahead of the camera
    assert not rig.call('charge_start').success  # standing
    assert rig.call('sit_down').success and station.sitting
    assert not rig.call('charge_start').success  # not on the contacts
    rig.place(station.contact)
    assert rig.call('charge_start').success and station.charging
    assert not rig.call('stand_up').success  # charging
    assert rig.call('charge_stop').success and rig.call('stand_up').success
    assert not station.sitting
    rig.drive(0.3, 1.0)
    response = rig.call('sit_down')
    assert not response.success and 'walking' in response.message


def test_charging_is_reported_when_asked(make):
    r = make(**{'battery.report_charging': True})
    station = r.node.sim.station
    r.place(station.contact)
    assert r.call('sit_down').success and r.call('charge_start').success
    charging = BatteryState.POWER_SUPPLY_STATUS_CHARGING
    last = lambda: r.battery.last  # noqa: E731
    assert wait_for(lambda: last() is not None and last().power_supply_status == charging, 3.0)


def test_stamps_carry_the_latency(make):
    """Scan, FAST-LIO and leg odometry stamps are the receipt time minus their latency."""
    r = make(**{'latency.lidar_sec': 0.3, 'latency.odometry_sec': 0.2})
    assert r.cloud.wait(3, 5.0) and r.lio.wait(3, 5.0) and r.odom.wait(3, 5.0)
    now = rospy.get_time()
    assert 0.3 <= now - to_sec(r.cloud.last.header.stamp) < 0.3 + 0.3
    assert wait_for(lambda: same_scan(r.lio.last, r.cloud.messages), 2.0)
    assert 0.2 <= now - to_sec(r.odom.last.header.stamp) < 0.2 + 0.3


def test_lidar_dropout_silences_fast_lio_too(make, tmp_path):
    """A scenario LiDAR dropout: no /livox/lidar, no FAST-LIO; IMU and leg odometry go on."""
    scenario = tmp_path / 'blind.yaml'
    scenario.write_text('seed: 9\ndropouts:\n  - {start: 0.0, duration: 100.0, sensor: lidar}\n')
    r = make(scenario_file=str(scenario))
    assert r.odom.wait(5, 5.0) and r.imu.wait(5, 5.0)
    assert not r.livox.messages and not r.lio.messages and not r.cloud.messages
    assert r.node.cfg.seed == 9


def test_leg_odometry_on_tf_with_publish_tf(make):
    r = make(publish_tf=True)
    assert wait_for(lambda: r.probe.tf.lookup('odom', 'base_link') is not None, 5.0)


def test_diff_robot_never_strafes(make):
    """robot.kinematics diff: a sideways command does not move the body."""
    r = make(**{'robot.kinematics': 'diff', 'robot.limits.max_vy': 0.0})
    assert r.truth.wait(1, 5.0)
    r.drive(0.0, 1.0, vy=0.5)
    assert r.truth.last.pose.position.y == pytest.approx(-3.0, abs=1e-6)
    assert math.isfinite(r.truth.last.pose.position.x)
