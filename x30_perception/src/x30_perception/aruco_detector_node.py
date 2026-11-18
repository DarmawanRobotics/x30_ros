"""aruco_detector_node: ArUco markers on a camera as TF frames, like apriltag_ros."""

from __future__ import annotations

from sensor_msgs.msg import CameraInfo, Image

from x30_perception.core.aruco import ArucoConfig, ArucoDetector, to_gray
from x30_utils.ros import SENSOR, X30Node, run

NAME = 'aruco_detector_node'


class ArucoDetectorNode(X30Node):
    """Detect ArUco markers and publish them on TF."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(ArucoConfig)
        self.detector = ArucoDetector(self.config.value)
        self.camera: tuple | None = None  # (K, D) from camera_info
        self.seen: set[int] = set()
        self.create_subscription(CameraInfo, 'camera_info', self.on_info, SENSOR)
        self.create_subscription(Image, 'image', self.on_image, SENSOR)
        self.every(5.0, self.watchdog, steady=True)
        self.log.info(
            'aruco ready', dictionary=self.config.dictionary, ids=str(list(self.config.ids)),
            frames=','.join(self.config.frames),
        )  # fmt: skip

    def on_info(self, msg: CameraInfo):
        """Keep the intrinsics (the image is rectified: distortion as given)."""
        self.camera = (list(msg.K), list(msg.D))

    def on_image(self, msg: Image):
        """Detect markers and broadcast camera -> tag for each one in the configuration."""
        if self.camera is None:
            self.log.warning('no camera_info yet', throttle=10.0)
            return
        try:
            gray = to_gray(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        except ValueError as error:
            self.log.error('image refused', error=str(error), throttle=10.0)
            return
        frame = msg.header.frame_id
        for marker in self.detector.detect(gray, *self.camera):
            self.tf_out.send(marker.matrix, frame, marker.frame, msg.header.stamp)
            if marker.id not in self.seen:
                self.seen.add(marker.id)
                self.log.info('marker seen', id=marker.id, frame=marker.frame)

    def watchdog(self):
        """Warn while no camera_info arrives."""
        if self.camera is None:
            self.log.warning('no camera_info: markers cannot be located', throttle=30.0)


def main():
    """Run aruco_detector_node."""
    run(ArucoDetectorNode, NAME)


if __name__ == '__main__':
    main()
