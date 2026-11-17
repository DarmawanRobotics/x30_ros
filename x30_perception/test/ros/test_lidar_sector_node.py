"""In-process tests of lidar_sector_node: Livox bytes, PointCloud2, depth camera, BLOCKED paths."""

from __future__ import annotations

import itertools
import math

from depth_scene import l1w_mount, render
from genpy.dynamic import generate_dynamic
from livox_pack import pack
import numpy as np
import pytest
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String

from x30_msgs.msg import SectorScan
from x30_perception.lidar_sector_node import LidarSectorNode
from x30_utils.config import ConfigError
from x30_utils.math import make_transform, quat_from_rpy
from x30_utils.ros import X30Node
from x30_utils.ros.pointcloud import xyz_cloud
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

MOUNT_Z = 0.2  # base_link -> livox_frame
BASE_HEIGHT = 0.35
CAMERA = l1w_mount()  # base_link -> camera_depth_optical_frame
LOW_BOX = (1.2, 1.5, -0.15, 0.15, 0.15)  # below the LiDAR, seen by the camera only
NAMES = itertools.count()
CUSTOM_MSG = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: livox_ros_driver2/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""


def room(n: int = 720, radius: float = 3.0, height: float = 0.3) -> np.ndarray:
    """A round room in livox_frame, height m above the floor."""
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    z = height - BASE_HEIGHT - MOUNT_Z
    return np.column_stack((radius * np.cos(a), radius * np.sin(a), np.full(n, z)))


def box(x: float, height: float = 0.6) -> np.ndarray:
    """Points of a box face x m ahead of the sensor."""
    z = height - BASE_HEIGHT - MOUNT_Z
    return np.array([[x, y, z] for y in np.linspace(-0.1, 0.1, 5)])


class Rig:
    """The node under test, TF and collectors in this process."""

    def __init__(self, odom_pitch: float = 0.0, **params):
        self.ns = f'/lidar_sector_test_{next(NAMES)}'
        defaults = {'input.timeout_sec': 3.0, 'robot.base_height': BASE_HEIGHT}
        self.node = LidarSectorNode(param_ns=self.ns, overrides={**defaults, **params})
        self.helper = X30Node('sector_test_helper', param_ns=f'{self.ns}_helper')
        self.helper.tf_out.send_static(
            [
                (make_transform((0.0, 0.0, MOUNT_Z)), 'base_link', 'livox_frame'),
                (
                    make_transform((1.0, 2.0, BASE_HEIGHT), quat_from_rpy(0.0, odom_pitch, 0.5)),
                    'odom',
                    'base_link',
                ),
                (CAMERA, 'base_link', 'camera_depth_optical_frame'),
            ]
        )
        self.front = Collector(SectorScan, 'perception/sector_scan')
        self.safety = Collector(SectorScan, 'perception/safety_scan')
        self.cloud_pub = rospy.Publisher('/test/cloud', PointCloud2, queue_size=10)
        self.depth_pub = rospy.Publisher('/test/depth', PointCloud2, queue_size=10)
        assert connected(self.front, self.safety)
        assert wait_for(
            lambda: (
                self.node.tf.static('base_link', 'livox_frame') is not None
                and self.node.tf.lookup('odom', 'base_link') is not None
            ),
            5.0,
        )

    def livox(self, points, **kw) -> tuple[SectorScan, SectorScan]:
        """Feed serialized CustomMsg bytes to the node's raw callback; return the next scans."""
        return self.raw(pack(points, **kw))

    def raw(self, data: bytes) -> tuple[SectorScan, SectorScan]:
        """Feed arbitrary bytes to the raw callback (under the node's callback lock)."""

        def send():
            with self.node._lock:
                self.node.on_livox(data)

        return self._next(send)

    def depth(self, points, count: int = 2):
        """Publish a depth cloud until the node filtered count more of them (a frame apart)."""
        header = Header(frame_id='camera_depth_optical_frame')
        assert connected(self.depth_pub)
        for _ in range(count):
            before = self.node._depth_last
            for _ in range(20):
                self.depth_pub.publish(xyz_cloud(header, points))
                if wait_for(lambda b=before: self.node._depth_last is not b, 0.5):
                    break
            wait_for(lambda: False, 1.0 / 15.0)

    def cloud(self, msg: PointCloud2) -> tuple[SectorScan, SectorScan]:
        """Publish a PointCloud2 on the input topic; return the next scans."""
        assert connected(self.cloud_pub)
        return self._next(lambda: self.cloud_pub.publish(msg))

    def _next(self, send) -> tuple[SectorScan, SectorScan]:
        before = len(self.safety.messages), len(self.front.messages)
        for _ in range(20):
            send()
            if wait_for(
                lambda: (
                    len(self.safety.messages) > before[0] and len(self.front.messages) > before[1]
                ),
                0.5,
            ):
                break
        return self.front.last, self.safety.last

    def close(self):
        self.node.destroy_node()
        self.helper.destroy_node()
        self.front.close()
        self.safety.close()
        self.cloud_pub.unregister()
        self.depth_pub.unregister()
        delete_params(self.ns)


@pytest.fixture
def make_rig(ros):
    rigs = []

    def factory(**kw) -> Rig:
        rig = Rig(**kw)
        rigs.append(rig)
        return rig

    yield factory
    for rig in rigs:
        rig.close()


@pytest.fixture
def rig(make_rig):
    """A rig with the default (livox) input."""
    return make_rig()


def blocked(scan: SectorScan) -> bool:
    """True when every sector is BLOCKED."""
    return len(scan.ranges) > 0 and all(r <= SectorScan.BLOCKED for r in scan.ranges)


def test_livox_raw_bytes_give_front_and_safety_scans(rig):
    """A box 1 m ahead in raw Livox bytes reaches both scans, in the robot base frame."""
    front, safety = rig.livox(np.vstack((room(), box(1.0))), sec=5, nanosec=7)
    assert front.header.frame_id == safety.header.frame_id == 'base_link'
    assert (front.header.stamp.secs, front.header.stamp.nsecs) == (5, 7)
    assert min(front.ranges) == pytest.approx(1.0, abs=0.01)
    assert len(safety.ranges) == 72 and safety.angle_min == pytest.approx(-math.pi)
    assert min(safety.ranges) == pytest.approx(1.0, abs=0.01)
    assert sorted(safety.ranges)[-1] == pytest.approx(3.0, abs=0.01)


def test_livox_topic_takes_the_drivers_type_and_drops_others(rig):
    """Over the topic: livox_ros_driver2/CustomMsg is read, another type is dropped."""
    custom = generate_dynamic('livox_ros_driver2/CustomMsg', CUSTOM_MSG)
    msg_type = custom['livox_ros_driver2/CustomMsg']
    livox = rospy.Publisher('livox/lidar', msg_type, queue_size=1)
    try:
        assert connected(livox)
        msg = msg_type()
        msg.deserialize(pack(np.vstack((room(), box(1.3)))))
        front, _ = rig._next(lambda: livox.publish(msg))
        assert min(front.ranges) == pytest.approx(1.3, abs=0.01)
    finally:
        livox.unregister()
    wrong = rospy.Publisher('livox/lidar', String, queue_size=1)
    try:
        assert connected(wrong)
        count = len(rig.front.messages)
        wrong.publish(String(data='not a cloud'))
        wait_for(lambda: False, 0.3)
        assert len(rig.front.messages) == count
    finally:
        wrong.unregister()


def test_livox_noise_tags_are_dropped(rig):
    """Points tagged low confidence (dust) do not block; the same points untagged do."""
    points = np.vstack((room(), box(1.0)))
    tags = np.zeros(len(points), np.uint8)
    tags[-5:] = 0b1000
    front, _ = rig.livox(points, tags=tags)
    assert min(front.ranges) == pytest.approx(3.0, abs=0.01)


def test_blind_livox_blocks_every_sector(rig):
    """A nearly empty cloud (covered LiDAR) publishes every sector BLOCKED."""
    front, safety = rig.livox(box(2.0))
    assert blocked(safety) and blocked(front)


def test_malformed_livox_bytes_keep_the_node_alive(rig):
    """Garbage and truncated buffers are dropped with an error; the next good scan works."""
    good = pack(np.vstack((room(), box(1.5))))
    for data in (b'', b'\x00\x01\x00\x00garbage', good[:50], good[:-7], b'\x07' * 300):
        with rig.node._lock:
            rig.node.on_livox(data)
    _, safety = rig.livox(np.vstack((room(), box(1.5))))
    assert min(safety.ranges) == pytest.approx(1.5, abs=0.01)


def test_unknown_sensor_frame_blocks(make_rig):
    """A cloud in a frame without TF to the base is BLOCKED, not read as free."""
    _, safety = make_rig().livox(room(), frame_id='unknown_lidar')
    assert blocked(safety)


def test_pitched_body_does_not_block_on_the_floor(make_rig):
    """Body pitched 3 deg down in odom: the floor seen by the LiDAR is not an obstacle."""
    pitch = math.radians(3.0)
    r = make_rig(odom_pitch=pitch)
    xs, ys = np.meshgrid(np.linspace(-6, 6, 61), np.linspace(-6, 6, 61))
    floor = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, -BASE_HEIGHT)))
    floor = floor[np.hypot(floor[:, 0], floor[:, 1]) > 1.0]
    tilt = make_transform(quat=quat_from_rpy(0.0, pitch, 0.0))[:3, :3]
    sensor = floor @ tilt - [0.0, 0.0, MOUNT_Z]
    front, safety = r.livox(sensor)
    assert not blocked(safety)
    assert all(v == pytest.approx(8.0) for v in front.ranges)
    assert all(v == pytest.approx(4.0) for v in safety.ranges)


def test_pointcloud_input_and_malformed_clouds(make_rig):
    """PointCloud2 input works; a malformed cloud is dropped and the node keeps running."""
    r = make_rig(**{'input.type': 'pointcloud2', 'input.pointcloud_topic': '/test/cloud'})
    header = Header(frame_id='livox_frame')
    front, _ = r.cloud(xyz_cloud(header, np.vstack((room(), box(1.2)))))
    assert min(front.ranges) == pytest.approx(1.2, abs=0.01)
    bad = xyz_cloud(header, room())
    bad.data = bytes(bad.data)[:100]
    count = len(r.safety.messages)
    r.cloud_pub.publish(bad)
    no_fields = PointCloud2(header=header, height=1, width=3, point_step=12, row_step=36)
    r.cloud_pub.publish(no_fields)
    wait_for(lambda: False, 0.3)
    assert len(r.safety.messages) == count  # dropped, nothing published
    _, safety = r.cloud(xyz_cloud(header, np.vstack((room(), box(2.2)))))
    assert min(safety.ranges) == pytest.approx(2.2, abs=0.01)


def test_silent_input_publishes_blocked_scans(make_rig):
    """No Livox driver / no data: the watchdog publishes BLOCKED scans after timeout_sec."""
    r = make_rig(**{'input.timeout_sec': 0.3})
    assert r.safety.wait(count=2, timeout=3.0)
    assert blocked(r.safety.last) and blocked(r.front.last)


def test_invalid_configuration_refuses_to_start(ros):
    """A bad safety value (inverted range limits) is rejected at start."""
    overrides = {'safety.range_min': 5.0, 'safety.range_max': 4.0}
    with pytest.raises(ConfigError, match='range_min'):
        LidarSectorNode(param_ns='/lidar_sector_bad', overrides=overrides)


def test_depth_camera_joins_the_scans_and_its_silence_falls_back(make_rig):
    """A box only the camera sees is in both scans; a silent camera leaves the LiDAR alone."""
    r = make_rig(**{'depth.enabled': True, 'depth.topic': '/test/depth', 'memory.enabled': False})
    _, safety = r.livox(room())
    assert min(safety.ranges) == pytest.approx(3.0, abs=0.15)  # LiDAR alone, not BLOCKED
    r.depth(render(CAMERA, boxes=[LOW_BOX]))
    front, safety = r.livox(room())
    assert min(front.ranges) == pytest.approx(1.2, abs=0.03)
    assert min(safety.ranges) == pytest.approx(1.2, abs=0.03)
    wait_for(lambda: False, 0.5)  # longer than depth.timeout_sec: the camera is silent
    front, safety = r.livox(room())
    assert not blocked(safety)
    assert min(front.ranges) > 2.5
    assert min(safety.ranges) == pytest.approx(1.2, abs=0.06)  # remembered, cell centres


def test_required_depth_camera_blocks_without_a_cloud(make_rig):
    """depth.required: no depth cloud yet, every sector BLOCKED; a cloud releases the scans."""
    r = make_rig(**{'depth.enabled': True, 'depth.topic': '/test/depth', 'depth.required': True})
    front, safety = r.livox(room())
    assert blocked(front) and blocked(safety)
    r.depth(render(CAMERA), count=1)
    _, safety = r.livox(room())
    assert not blocked(safety)
