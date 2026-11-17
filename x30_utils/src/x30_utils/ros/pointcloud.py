"""PointCloud2 <-> numpy, validated: a malformed cloud raises CloudError instead of crashing."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

_DTYPES = {PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


class CloudError(ValueError):
    """The message is not a usable xyz cloud."""


def read_xyz(msg: PointCloud2) -> np.ndarray:
    """Return the finite points of a PointCloud2 as (N, 3) float32."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in 'xyz'):
        raise CloudError('cloud has no x, y, z fields')
    kinds = {fields[k].datatype for k in 'xyz'}
    if len(kinds) != 1 or next(iter(kinds)) not in _DTYPES:
        raise CloudError(f'unsupported x, y, z datatype {sorted(kinds)}')
    scalar = np.dtype(_DTYPES[kinds.pop()]).newbyteorder('>' if msg.is_bigendian else '<')
    n, step, row = msg.width * msg.height, msg.point_step, msg.row_step
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[k].offset + scalar.itemsize > step for k in 'xyz'):
        raise CloudError('x, y or z lies outside the point')
    if row < msg.width * step or len(msg.data) < row * (msg.height - 1) + msg.width * step:
        raise CloudError(f'cloud data too short: {len(msg.data)} bytes for {n} points')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if row != msg.width * step:
        raw = np.concatenate(
            [raw[i * row : i * row + msg.width * step] for i in range(msg.height)]
        )
    point = np.dtype(
        {
            'names': ['x', 'y', 'z'],
            'formats': [scalar] * 3,
            'offsets': [fields[k].offset for k in 'xyz'],
            'itemsize': step,
        }
    )
    cloud = np.frombuffer(raw[: n * step].tobytes(), dtype=point, count=n)
    xyz = np.column_stack((cloud['x'], cloud['y'], cloud['z'])).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_cloud(header, points: np.ndarray) -> PointCloud2:
    """Return a dense float32 xyz PointCloud2."""
    points = np.ascontiguousarray(np.asarray(points, dtype=np.float32).reshape(-1, 3))
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(points)
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate('xyz')
    ]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg
