"""Acoustic chunks: rows gathered for chunk_sec, variable-length fields with offsets, no pickle."""

from __future__ import annotations

import numpy as np

from x30_recorder.core.chunks import Chunk, row, write_arrays


def test_rows_become_arrays_once_the_chunk_is_due(tmp_path):
    chunk = Chunk(('bin_hz', 'band_peak'), ('magnitude',), chunk_sec=10.0)
    assert chunk.take() is None and not chunk.due(5.0)
    spectra = [np.arange(4, dtype=np.int16), np.arange(6, dtype=np.int16) * 2]
    for i, magnitude in enumerate(spectra):
        chunk.add(100.0 + 5.0 * i, bin_hz=5.0, band_peak=10 + i, magnitude=magnitude)
    assert not chunk.due(109.9) and chunk.due(110.0)
    arrays = chunk.take()
    assert chunk.take() is None  # started over
    assert arrays['stamp'].tolist() == [100.0, 105.0] and arrays['band_peak'].tolist() == [10, 11]
    path = str(tmp_path / 'acoustic' / 'spectrum.npz')
    write_arrays(path, arrays)
    with np.load(path, allow_pickle=False) as back:
        for i, magnitude in enumerate(spectra):
            assert np.array_equal(row(back, 'magnitude', i), magnitude)
        assert back['magnitude'].dtype == np.int16


def test_raw_bytes_keep_their_values(tmp_path):
    chunk = Chunk((), ('data',), chunk_sec=1.0)
    frames = [bytes(range(256)), b'', b'\x00\xff\x10']
    for i, data in enumerate(frames):
        chunk.add(float(i), data=np.frombuffer(data, dtype=np.uint8))
    path = str(tmp_path / 'prpd.npz')
    write_arrays(path, chunk.take())
    with np.load(path, allow_pickle=False) as back:
        assert [bytes(row(back, 'data', i)) for i in range(3)] == frames
