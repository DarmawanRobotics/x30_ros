"""A synthetic camera image of an ArUco marker (import after importorskip('cv2'))."""

from __future__ import annotations

import cv2
import numpy as np

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
SIZE = 0.15


def marker_image(marker_id: int, pixels: int = 200, pad: int = 50) -> np.ndarray:
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'generateImageMarker'):
        img = aruco.generateImageMarker(dictionary, marker_id, pixels)
    else:  # OpenCV < 4.7
        img = aruco.drawMarker(dictionary, marker_id, pixels)
    return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def render(marker_id: int, rotation: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Return a 640x480 mono8 image of the marker at rotation, t (OpenCV marker frame)."""
    h = SIZE / 2.0
    corners = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    cam = corners @ rotation.T + t
    pix = (cam @ K.T)[:, :2] / cam[:, 2:3]
    src = np.array([[50, 50], [250, 50], [250, 250], [50, 250]], dtype=np.float32)
    warp = cv2.getPerspectiveTransform(src, pix.astype(np.float32))
    return cv2.warpPerspective(marker_image(marker_id), warp, (640, 480), borderValue=255)


def yaw_rotation(angle: float) -> np.ndarray:
    """A marker facing the camera (x right, y up, z toward it), turned about the image y axis."""
    facing = np.diag([1.0, -1.0, -1.0])
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]) @ facing
