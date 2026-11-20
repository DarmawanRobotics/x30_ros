"""The wire format: chunked streams, packed points, sticks that cannot turn into a speed."""

from __future__ import annotations

import random

import numpy as np

from x30_bridge.core import protocol
from x30_utils.math import default_rng


def test_chunks_reassemble_out_of_order():
    payload = bytes(random.getrandbits(8) for _ in range(9000))
    datagrams = protocol.chunk(protocol.STREAM_CAMERA, 5, payload)
    assert all(len(d) <= protocol.HEADER.size + protocol.CHUNK_PAYLOAD for d in datagrams)
    random.shuffle(datagrams)
    r = protocol.Reassembler()
    done = [x for x in (r.push(d) for d in datagrams) if x]
    assert done == [(protocol.STREAM_CAMERA, 5, payload)]


def test_an_incomplete_frame_is_dropped_for_a_newer_one():
    r = protocol.Reassembler()
    for d in protocol.chunk(1, 1, bytes(5000))[:-1]:
        assert r.push(d) is None
    assert r.push(protocol.chunk(1, 2, b'ok')[0]) == (1, 2, b'ok')
    assert r.push(b'XXXX' + bytes(20)) is None  # not a stream of this protocol


def test_points_roundtrip_within_half_a_centimetre_and_downsample():
    xyz = default_rng(1).uniform(-20, 20, (500, 3)).astype(np.float32)
    assert np.abs(protocol.unpack_points(protocol.pack_points(xyz)) - xyz).max() <= 0.005 + 1e-6
    many = default_rng(2).uniform(-5, 5, (20000, 3)).astype(np.float32)
    assert protocol.voxel_downsample(many, 0.1, 1000).shape[0] == 1000
    assert protocol.voxel_downsample(np.zeros((0, 3)), 0.1, 10).shape == (0, 3)


def test_a_stick_axis_is_clamped_and_garbage_is_zero():
    assert protocol.clamp_axis(0.4, 1.0) == 0.4
    assert protocol.clamp_axis(5.0, 1.0) == 1.0 and protocol.clamp_axis(-5.0, 1.0) == -1.0
    assert protocol.clamp_axis('0.3', 1.0) == 0.3
    for bad in (float('nan'), float('inf'), float('-inf'), None, 'fast', [1], {}, True):
        assert protocol.clamp_axis(bad, 1.0) == 0.0


def test_a_message_is_a_json_object_with_a_type():
    assert protocol.decode_json(b'{"t": "ping"}') == {'t': 'ping'}
    for bad in (b'', b'\xff\xfe', b'[1, 2]', b'{"x": 1}', b'not json'):
        assert protocol.decode_json(bad) is None
    assert protocol.decode_json(protocol.encode_json({'t': 'x', 'v': [1]})) == {'t': 'x', 'v': [1]}


def test_a_long_route_keeps_both_ends():
    route = [[float(i), 0.0, 0.0] for i in range(1000)]
    short = protocol.simplify_route(route, 600)
    assert len(short) == 600 and short[0] == route[0] and short[-1] == route[-1]
    assert protocol.simplify_route(route[:3], 600) == route[:3]
