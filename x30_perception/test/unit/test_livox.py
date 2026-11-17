"""Livox CustomMsg parsing (ROS 1): documented layout, malformed buffers (fuzz), tag-bit noise."""

from __future__ import annotations

from io import BytesIO
import struct

from livox_pack import pack
import numpy as np
import pytest

from x30_perception.core.livox import LivoxError, LivoxScan, noise_mask, parse_custom_msg
from x30_utils.math import default_rng

POINTS = np.array(
    [[1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [-4.5, 0.25, -0.5], [np.nan, 1.0, 1.0], [7.0, -8.0, 0.1]],
    dtype=np.float32,
)
VALID = POINTS[[0, 2, 4]]
FRAME = 16  # first byte of frame_id


def test_packer_matches_the_documented_layout():
    """Byte offsets of a two-point message, worked out by hand from the message definition."""
    data = pack([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], 'livox_frame', sec=7, nanosec=9, tags=[3, 16])
    assert struct.unpack_from('<IIII', data, 0) == (0, 7, 9, 11)  # seq, secs, nsecs, len
    assert data[16:27] == b'livox_frame'
    assert struct.unpack_from('<QIB', data, 27) == (42, 2, 1)  # timebase, point_num, lidar_id
    assert data[40:43] == b'\x00\x00\x00'  # rsvd
    assert struct.unpack_from('<I', data, 43) == (2,)  # array length
    assert struct.unpack_from('<IfffBBB', data, 47) == (0, 1.0, 2.0, 3.0, 5, 3, 1)
    assert struct.unpack_from('<IfffBBB', data, 66) == (1, 4.0, 5.0, 6.0, 5, 16, 1)  # stride 19
    assert len(data) == 66 + 19


def test_header_points_and_tags():
    """Stamp, frame, points and tags come back; points at the origin or with NaN are dropped."""
    scan = parse_custom_msg(pack(POINTS, tags=[1, 2, 3, 4, 5]))
    assert isinstance(scan, LivoxScan)
    assert (scan.sec, scan.nanosec, scan.frame_id) == (12, 34, 'livox_frame')
    assert scan.stamp == pytest.approx(12.000000034)
    assert scan.xyz.dtype == np.float32 and scan.xyz.shape == (3, 3)
    np.testing.assert_array_equal(scan.xyz, VALID)
    assert scan.tags.tolist() == [1, 3, 5]


@pytest.mark.parametrize('frame_id', ['', 'a', 'abc', 'abcd', 'lidar_frame_x', 'x' * 8])
def test_any_frame_name_length(frame_id):
    """Every frame_id length (also none) moves the rest of the message, unaligned."""
    scan = parse_custom_msg(pack(POINTS, frame_id))
    assert scan.frame_id == frame_id
    np.testing.assert_array_equal(scan.xyz, VALID)


def test_empty_cloud():
    """A scan without points parses to an empty Nx3 array."""
    scan = parse_custom_msg(pack(POINTS[:0]))
    assert scan.xyz.shape == (0, 3) and scan.tags.shape == (0,)


@pytest.mark.parametrize(
    'data',
    [
        b'',
        b'\x00\x01',
        pack(POINTS)[:-5],
        pack(POINTS)[:30],
        pack(POINTS)[:10],
        pack(POINTS) + b'\x00',
    ],
)
def test_broken_buffers_are_refused(data):
    """A truncated or overlong buffer raises LivoxError instead of returning garbage."""
    with pytest.raises(LivoxError):
        parse_custom_msg(data)


def test_huge_point_count_is_refused():
    """A corrupted point count never allocates or reads past the buffer."""
    data = bytearray(pack(POINTS))
    struct.pack_into('<I', data, FRAME + len('livox_frame') + 16, 0xFFFFFFFF)
    with pytest.raises(LivoxError):
        parse_custom_msg(bytes(data))


def test_frame_id_length_out_of_range_or_not_text_is_refused():
    """A frame_id length beyond the buffer, or bytes that are not UTF-8, are refused."""
    data = bytearray(pack(POINTS))
    struct.pack_into('<I', data, 12, 100000)
    with pytest.raises(LivoxError):
        parse_custom_msg(bytes(data))
    data = bytearray(pack(POINTS))
    data[FRAME] = 0xFF
    with pytest.raises(LivoxError):
        parse_custom_msg(bytes(data))


def test_fuzz_never_raises_anything_else():
    """Random truncations, byte flips and noise give a LivoxScan or LivoxError, nothing else."""
    rng = default_rng(7)
    good = pack(rng.uniform(-5, 5, (40, 3)).astype(np.float32))
    for _ in range(3000):
        data = bytearray(good)
        mode = rng.integers(3)
        if mode == 0:
            data = data[: rng.integers(len(data))]
        elif mode == 1:
            for i in rng.integers(0, len(data), rng.integers(1, 8)):
                data[i] = rng.integers(256)
        else:
            data = bytearray(rng.integers(0, 256, rng.integers(0, 200)).astype(np.uint8).tobytes())
        try:
            scan = parse_custom_msg(bytes(data))
        except LivoxError:
            continue
        assert scan.xyz.shape[1] == 3 and len(scan.tags) == len(scan.xyz)
        assert np.isfinite(scan.xyz).all()


def test_noise_mask_selects_fields_and_levels():
    """Low confidence in a selected field is noise; medium only at level 1; reserved is kept."""
    tags = np.array([0b00, 0b10, 0b01, 0b11, 0b1000, 0b100000, 0b10_00_00, 0b0100], np.uint8)
    adhesion = noise_mask(tags, ('adhesion',))
    assert adhesion.tolist() == [False, True, False, False, False, False, False, False]
    both = noise_mask(tags, ('adhesion', 'particle'))
    assert both.tolist() == [False, True, False, False, True, False, False, False]
    medium = noise_mask(tags, ('adhesion', 'particle'), level=1)
    assert medium.tolist() == [False, True, True, False, True, False, False, True]
    assert noise_mask(tags, ('other',)).tolist()[5:7] == [True, True]
    assert not noise_mask(tags, ()).any()


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


def test_matches_the_genpy_serializer():
    """Parity with ROS 1's own serializer, on the driver's message definition."""
    genpy = pytest.importorskip('genpy')
    from genpy.dynamic import generate_dynamic

    types = generate_dynamic('livox_ros_driver2/CustomMsg', CUSTOM_MSG)
    custom_point = types['livox_ros_driver2/CustomPoint']
    msg = types['livox_ros_driver2/CustomMsg'](timebase=42, lidar_id=1, rsvd=[0, 0, 0])
    msg.header.stamp = genpy.Time(1700000000, 999)
    msg.header.frame_id = 'livox_frame'
    msg.points = [
        custom_point(offset_time=i, x=float(x), y=float(y), z=float(z), reflectivity=5, line=1)
        for i, (x, y, z) in enumerate(POINTS)
    ]
    msg.point_num = len(msg.points)
    out = BytesIO()
    msg.serialize(out)
    assert out.getvalue() == pack(POINTS, sec=1700000000, nanosec=999)
