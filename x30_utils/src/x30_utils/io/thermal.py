"""Raw thermal frames as .npz: the temperatures (degC) and what was known when they were taken."""

# One frame per file, written atomically and synced: a crash or a reader never sees half a
# frame. Arrays (np.load(path) needs no pickle):
#   temperature  float32 (H, W), degC per pixel
#   stamp        float64, the frame's time (s, ROS time)
#   pose         float64 (x, y, yaw), the robot in the map frame; NaN: not known
#   ptz          float64 (pan, tilt, zoom) of the camera, degrees and times; NaN: not known
#   meta         a JSON object as a string: frame_id, site, the camera's thermometry, ...

from __future__ import annotations

import contextlib
import json
import math
import os
import tempfile

import numpy as np

UNKNOWN = (math.nan, math.nan, math.nan)


def image_temperatures(
    height: int, width: int, step: int, data: bytes, bigendian: bool = False
) -> np.ndarray:
    """Return the (H, W) float32 degC of a 32FC1 image's fields (rows may be padded)."""
    if height <= 0 or width <= 0 or step < 4 * width or len(data) < height * step:
        raise ValueError(f'not a {width} x {height} 32FC1 image: step {step}, {len(data)} bytes')
    dtype = np.dtype('>f4' if bigendian else '<f4')
    rows = np.frombuffer(data, dtype=dtype, count=height * step // 4)
    rows = rows.reshape(height, step // 4)[:, :width]
    return np.ascontiguousarray(rows, dtype=np.float32)


def write_thermal(path: str, temperature, stamp: float, pose=UNKNOWN, ptz=UNKNOWN, meta=None):
    """Write one raw thermal frame to path (.npz, compressed), atomically."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp-', suffix='.npz', dir=directory)
    try:
        with os.fdopen(fd, 'wb') as f:
            np.savez_compressed(
                f,
                temperature=np.asarray(temperature, dtype=np.float32),
                stamp=np.float64(stamp),
                pose=np.asarray(pose, dtype=np.float64).reshape(3),
                ptz=np.asarray(ptz, dtype=np.float64).reshape(3),
                meta=np.array(json.dumps(meta or {}, sort_keys=True, allow_nan=False)),
            )
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp)
        raise


def read_thermal(path: str) -> dict:
    """Return a frame of write_thermal: temperature, stamp, pose, ptz, meta (a dict)."""
    with np.load(path, allow_pickle=False) as data:
        return {
            'temperature': data['temperature'],
            'stamp': float(data['stamp']),
            'pose': data['pose'],
            'ptz': data['ptz'],
            'meta': json.loads(str(data['meta'])),
        }
