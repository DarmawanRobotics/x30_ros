"""The handheld's camera frames, cloud and pose, subscribed only while it uses them."""

# Deserializing a camera, a cloud and every TF message costs CPU on the computer that drives:
# nothing is subscribed while no handheld is connected.

from __future__ import annotations

import time

import numpy as np
from sensor_msgs.msg import CompressedImage, PointCloud2

from x30_bridge.core import protocol
from x30_bridge.core.commands import camera_period
from x30_utils.math import yaw_of
from x30_utils.ros import SENSOR
from x30_utils.ros.pointcloud import CloudError, read_xyz
from x30_utils.ros.tf import TfCache


def camera_topics(items) -> dict:
    """Return {name: topic} of the cameras parameter (name=topic each, in order)."""
    out = {}
    for item in items:
        name, _, topic = item.partition('=')
        out[name] = topic
    return out


class Streams:
    """JPEG frames of one camera at a time and downsampled clouds for one handheld."""

    def __init__(self, node, config, base_frame: str, send_stream):
        self.node, self.config, self.base = node, config, base_frame
        self.send_stream = send_stream  # (stream id, payload)
        self.cameras = camera_topics(config.cameras)
        self.camera = next(iter(self.cameras), '')
        self.wanted = {'camera': False, 'lidar': False, 'video': False}
        self.subs: dict = {}
        self.tf: TfCache | None = None
        self.camera_period = 1.0 / config.camera_fps
        self.lidar_period = 1.0 / config.lidar_hz
        self.last_camera = self.last_lidar = 0.0
        self.warned = False

    def update(self, values: dict):
        """Apply a handheld's streams choice (camera, lidar, video, camera_fps, camera_name)."""
        self.wanted.update({k: bool(v) for k, v in values.items() if k in self.wanted})
        period = camera_period(values.get('camera_fps'), self.config.camera_fps)
        if period is not None:
            self.camera_period = period
        name = values.get('camera_name')
        if isinstance(name, str) and name in self.cameras and name != self.camera:
            self.drop('camera')
            self.camera = name

    def sync(self, live: bool):
        """Hold the stream and TF subscriptions only while a client uses them."""
        if live and self.tf is None:
            self.tf = TfCache(self.node)
        elif not live and self.tf is not None:
            self.tf.close()
            self.tf = None
        sources = {
            'camera': (CompressedImage, self.cameras.get(self.camera, ''), self.on_camera),
            'lidar': (PointCloud2, self.config.lidar_topic, self.on_lidar),
        }
        for name, (kind, topic, callback) in sources.items():
            want = live and self.wanted[name] and bool(topic)
            if want and name not in self.subs:
                self.subs[name] = self.node.create_subscription(kind, topic, callback, SENSOR)
            elif not want:
                self.drop(name)

    def drop(self, name: str):
        """Unsubscribe one stream."""
        sub = self.subs.pop(name, None)
        if sub is not None:
            self.node.destroy_subscription(sub)

    def pose(self, map_frame: str):
        """Return the robot [x, y, yaw] in the map frame, or None."""
        found = self.tf.lookup(map_frame, self.base) if self.tf is not None else None
        if found is None:
            return None
        m = found.matrix
        return [round(float(m[0, 3]), 3), round(float(m[1, 3]), 3), round(yaw_of(m), 3)]

    # ---------------------------------------------------------------- camera
    def on_camera(self, msg: CompressedImage):
        """Forward a throttled JPEG frame."""
        now = time.monotonic()
        if not self.wanted['camera'] or now - self.last_camera < self.camera_period:
            return
        self.last_camera = now
        jpeg = self.shrink(bytes(msg.data))
        if jpeg:
            self.send_stream(protocol.STREAM_CAMERA, jpeg)

    def shrink(self, jpeg: bytes) -> bytes:
        """Re-encode a JPEG bigger than camera_max_bytes (one lost datagram loses a frame)."""
        if len(jpeg) <= self.config.camera_max_bytes:
            return jpeg
        try:
            import cv2
        except ImportError:
            if not self.warned:
                self.warned = True
                self.node.log.warning('camera frames too big for the handheld and no OpenCV',
                                      bytes=len(jpeg))  # fmt: skip
            return jpeg
        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return b''
        height, width = img.shape[:2]
        limit = self.config.camera_max_width
        if width > limit:
            img = cv2.resize(
                img, (limit, int(height * limit / width)), interpolation=cv2.INTER_AREA
            )
        ok, buf = cv2.imencode(
            '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.config.camera_jpeg_quality]
        )
        return buf.tobytes() if ok else b''

    # ---------------------------------------------------------------- lidar
    def on_lidar(self, msg: PointCloud2):
        """Forward a throttled, voxel-downsampled cloud in the robot frame."""
        now = time.monotonic()
        if not self.wanted['lidar'] or now - self.last_lidar < self.lidar_period:
            return
        self.last_lidar = now
        extrinsic = self.tf.static(self.base, msg.header.frame_id) if self.tf is not None else None
        if extrinsic is None:
            return
        try:
            xyz = read_xyz(msg)
        except CloudError:
            return
        xyz = (xyz @ extrinsic[:3, :3].T + extrinsic[:3, 3]).astype(np.float32)
        c = self.config
        xyz = protocol.voxel_downsample(xyz, c.lidar_voxel, c.lidar_max_points)
        self.send_stream(protocol.STREAM_LIDAR, protocol.pack_points(xyz))
