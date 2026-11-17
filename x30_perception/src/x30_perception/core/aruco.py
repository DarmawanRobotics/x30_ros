"""ArUco detection and pose with OpenCV 4.2 to 4.7+, as camera -> tag transforms."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from x30_utils.config import param

ENCODINGS = ('mono8', 'rgb8', 'bgr8', 'rgba8', 'bgra8')
TO_STACK = np.diag([-1.0, -1.0, 1.0, 1.0])  # OpenCV marker frame -> stack tag frame


@dataclass(frozen=True)
class ArucoConfig:
    """aruco_detector_node parameters: the dictionary and the markers published on TF."""

    dictionary: str = param('DICT_4X4_50', live=False, doc='cv2.aruco predefined dictionary')
    ids: tuple[int, ...] = param((4,), live=False, doc='marker ids published as TF frames')
    frames: tuple[str, ...] = param(('charging_dock',), live=False, doc='TF frame per id')
    sizes: tuple[float, ...] = param((0.15,), unit='m', live=False, doc='edge length per id')
    max_distance: float = param(
        6.0, min=0.1, max=50.0, unit='m', doc='poses further than this are dropped'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if not len(self.ids) == len(self.frames) == len(self.sizes):
            problems.append('ids, frames and sizes need one entry per marker')
        if any(s <= 0.0 for s in self.sizes):
            problems.append('sizes must be positive')
        if len(set(self.ids)) != len(self.ids):
            problems.append('ids must be unique')
        return problems


@dataclass(frozen=True)
class MarkerPose:
    """One detected marker: id, its TF frame and the 4x4 camera -> tag transform."""

    id: int
    frame: str
    matrix: np.ndarray


def to_gray(data: bytes, width: int, height: int, step: int, encoding: str) -> np.ndarray:
    """Return a mono8 image from raw sensor_msgs/Image data (ValueError: unsupported)."""
    if encoding not in ENCODINGS:
        raise ValueError(f'unsupported encoding {encoding!r}, use one of {ENCODINGS}')
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}[encoding]
    rows = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels).astype(np.float32)
    if channels == 1:
        return img[:, :, 0].astype(np.uint8)
    r, g, b = (0, 1, 2) if encoding.startswith('rgb') else (2, 1, 0)
    gray = 0.299 * img[:, :, r] + 0.587 * img[:, :, g] + 0.114 * img[:, :, b]
    return np.clip(gray + 0.5, 0, 255).astype(np.uint8)


class ArucoDetector:
    """Detect the configured markers and estimate their poses (needs cv2 with aruco)."""

    def __init__(self, config: ArucoConfig):
        import cv2  # imported here: a node without markers configured needs no OpenCV

        self.cv2, self.config = cv2, config
        aruco = cv2.aruco
        if not hasattr(aruco, config.dictionary):
            raise ValueError(f'unknown ArUco dictionary {config.dictionary!r}')
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, config.dictionary))
        if hasattr(aruco, 'ArucoDetector'):  # OpenCV 4.7+
            self._detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
            self._detect = self._detector.detectMarkers
        else:
            params = aruco.DetectorParameters_create()
            self._detect = lambda img: aruco.detectMarkers(img, self.dictionary, parameters=params)
        self.markers = {
            int(i): (f, float(s)) for i, f, s in zip(config.ids, config.frames, config.sizes)
        }

    def detect(self, gray: np.ndarray, camera_matrix, distortion) -> list[MarkerPose]:
        """Return the poses of the configured markers seen in a mono8 image."""
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return []
        k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        d = np.asarray(distortion if distortion is not None else [], dtype=np.float64)
        out = []
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            entry = self.markers.get(int(marker_id))
            if entry is None:
                continue
            matrix = self._pose(np.asarray(marker_corners).reshape(4, 2), entry[1], k, d)
            if matrix is not None and np.linalg.norm(matrix[:3, 3]) <= self.config.max_distance:
                out.append(MarkerPose(int(marker_id), entry[0], matrix))
        return out

    def _pose(self, corners: np.ndarray, size: float, k, d) -> np.ndarray | None:
        cv2, h = self.cv2, size / 2.0
        # corner order of the detector: top-left, top-right, bottom-right, bottom-left
        obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
        ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float64), k, d,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)  # fmt: skip
        t = np.asarray(tvec, dtype=float).reshape(3)
        if not ok or not np.all(np.isfinite(t)) or t[2] <= 0.0:
            return None
        m = np.eye(4)
        m[:3, :3], _ = cv2.Rodrigues(rvec)
        m[:3, 3] = t
        return m @ TO_STACK
