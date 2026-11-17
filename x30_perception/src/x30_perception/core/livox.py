"""Read a serialized ROS 1 Livox CustomMsg from its bytes, without per-point objects."""

from __future__ import annotations

from dataclasses import dataclass
import struct

import numpy as np

# ROS 1 serialization (little endian, no padding): header (uint32 seq, uint32 secs, uint32
# nsecs, string frame_id as uint32 length + bytes), uint64 timebase, uint32 point_num, uint8
# lidar_id, uint8[3] rsvd, then uint32 count and CustomPoints of uint32 offset_time, float32
# x y z, uint8 reflectivity tag line, 19 bytes each. livox_ros_driver and livox_ros_driver2
# serialize it the same. Tag bits 0-1 adhesion, 2-3 rain / fog / dust, 4-5 other: 0 high
# confidence, 1 medium, 2 low, 3 reserved (kept: the safe side).
HEADER = struct.Struct('<IIII')
AFTER_FRAME = struct.Struct('<QIB3xI')  # timebase, point_num, lidar_id, rsvd, count
POINT_SIZE = 19
XYZ_OFFSET = 4
TAG_OFFSET = 17
MAX_FRAME_ID = 256
TAG_FIELDS = {'adhesion': 0, 'particle': 2, 'other': 4}
TYPES = ('livox_ros_driver2/CustomMsg', 'livox_ros_driver/CustomMsg')
"""The message types the bytes may come from (the publisher's connection header)."""


class LivoxError(ValueError):
    """The bytes are not a usable serialized CustomMsg."""


@dataclass(frozen=True)
class LivoxScan:
    """Header and valid points of one CustomMsg (sensor frame)."""

    sec: int
    nanosec: int
    frame_id: str
    xyz: np.ndarray
    tags: np.ndarray

    @property
    def stamp(self) -> float:
        """Header stamp in seconds."""
        return self.sec + self.nanosec * 1e-9


def _header(buf: memoryview) -> tuple[int, int, str, int, int]:
    """Return (secs, nsecs, frame_id, point count, offset of the first point)."""
    if len(buf) < HEADER.size:
        raise LivoxError(f'truncated header ({len(buf)} bytes)')
    _seq, sec, nanosec, name_len = HEADER.unpack_from(buf, 0)
    if name_len > MAX_FRAME_ID or HEADER.size + name_len + AFTER_FRAME.size > len(buf):
        raise LivoxError(f'frame_id length {name_len} does not fit in {len(buf)} bytes')
    raw_name = bytes(buf[HEADER.size : HEADER.size + name_len])
    try:
        frame_id = raw_name.decode()
    except UnicodeDecodeError as error:
        raise LivoxError('frame_id is not text') from error
    offset = HEADER.size + name_len
    *_, count = AFTER_FRAME.unpack_from(buf, offset)
    return sec, nanosec, frame_id, count, offset + AFTER_FRAME.size


def parse_custom_msg(data: bytes) -> LivoxScan:
    """Return stamp, frame and valid points of a serialized CustomMsg; raise LivoxError."""
    buf = memoryview(data)
    sec, nanosec, frame_id, count, offset = _header(buf)
    if offset + count * POINT_SIZE != len(buf):
        raise LivoxError(f'{count} points do not match {len(buf)} bytes')
    if count == 0:
        empty = np.empty((0, 3), np.float32)
        return LivoxScan(sec, nanosec, frame_id, empty, np.empty(0, np.uint8))
    xyz = np.ndarray(
        (count, 3),
        dtype='<f4',
        buffer=buf,
        offset=offset + XYZ_OFFSET,
        strides=(POINT_SIZE, 4),
    )
    tags = np.ndarray(
        (count,), dtype=np.uint8, buffer=buf, offset=offset + TAG_OFFSET, strides=(POINT_SIZE,)
    )
    with np.errstate(invalid='ignore'):
        keep = np.isfinite(xyz).all(axis=1) & (xyz != 0.0).any(axis=1)
    return LivoxScan(sec, nanosec, frame_id, xyz[keep].astype(np.float32), tags[keep].copy())


def noise_mask(tags: np.ndarray, fields: tuple[str, ...], level: int = 2) -> np.ndarray:
    """Return True for points with a selected tag field at confidence >= level."""
    tags = np.asarray(tags, dtype=np.uint8)
    noisy = np.zeros(tags.shape, dtype=bool)
    for name in fields:
        value = (tags >> TAG_FIELDS[name]) & 0b11
        noisy |= (value >= level) & (value <= 2)
    return noisy
