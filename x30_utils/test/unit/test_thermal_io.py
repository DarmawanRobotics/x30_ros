"""Raw thermal frames: 32FC1 fields to degC, one atomic .npz per frame, read back as written."""

from __future__ import annotations

import math
import os

import numpy as np
import pytest

from x30_utils.io import image_temperatures, read_thermal, write_thermal


def test_a_padded_little_or_big_endian_image_reads_as_its_temperatures():
    t = np.array([[20.5, 21.0, 22.25], [30.0, -5.5, 100.0]], dtype=np.float32)
    for bigendian in (False, True):
        raw = t.astype('>f4' if bigendian else '<f4')
        padded = np.zeros((2, 4), dtype=raw.dtype)  # a step of 16 bytes for 3 pixels
        padded[:, :3] = raw
        out = image_temperatures(2, 3, 16, padded.tobytes(), bigendian)
        assert out.dtype == np.float32 and out.flags['C_CONTIGUOUS']
        assert np.array_equal(out, t)
    with pytest.raises(ValueError):
        image_temperatures(2, 3, 8, padded.tobytes())  # a row shorter than its pixels


def test_a_frame_is_written_whole_and_read_back(tmp_path):
    t = np.random.default_rng(1).normal(35.0, 4.0, size=(512, 640)).astype(np.float32)
    path = str(tmp_path / 'thermal' / 'frame.npz')
    meta = {'frame_id': 'hikmicro_thermal_optical_frame', 'emissivity': 0.95}
    write_thermal(
        path, t, 1700000000.25, pose=(1.0, -2.0, 0.5), ptz=(120.0, -10.0, 4.0), meta=meta
    )
    assert os.listdir(tmp_path / 'thermal') == ['frame.npz']  # no temporary file left
    frame = read_thermal(path)
    assert np.array_equal(frame['temperature'], t) and frame['stamp'] == 1700000000.25
    assert frame['pose'].tolist() == [1.0, -2.0, 0.5] and frame['ptz'].tolist() == [
        120.0,
        -10.0,
        4.0,
    ]
    assert frame['meta'] == meta
    unknown = str(tmp_path / 'unknown.npz')
    write_thermal(unknown, t[:2, :2], 1.0)
    assert all(math.isnan(v) for v in read_thermal(unknown)['pose'])
    with np.load(unknown, allow_pickle=False) as data:  # plain arrays: no pickle to load it
        assert set(data.files) == {'temperature', 'stamp', 'pose', 'ptz', 'meta'}
