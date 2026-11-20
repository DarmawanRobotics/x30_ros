"""Rows of a stream gathered for a while and written as one .npz: spectra, raw frames."""

# Scalars are stacked into (N,) arrays; variable-length fields (a spectrum's magnitudes, a raw
# frame's bytes) are concatenated with an <field>_offsets array of N + 1 indices: row i is
# field[offsets[i]:offsets[i + 1]]. No pickle is needed to read them back.

from __future__ import annotations

import contextlib
import os
import tempfile

import numpy as np


class Chunk:
    """Rows of one stream; take() returns them as arrays once chunk_sec passed (or on flush)."""

    def __init__(self, scalars: tuple[str, ...], arrays: tuple[str, ...], chunk_sec: float):
        self.scalars, self.arrays, self.chunk_sec = scalars, arrays, chunk_sec
        self.rows: list[dict] = []
        self.first = 0.0

    def add(self, stamp: float, **row):
        """Add a row taken at stamp (s)."""
        if not self.rows:
            self.first = stamp
        self.rows.append({'stamp': stamp, **row})

    def due(self, now: float) -> bool:
        """Return True when the rows span chunk_sec (by the clock of their stamps)."""
        return bool(self.rows) and now - self.first >= self.chunk_sec

    def take(self) -> dict | None:
        """Return the rows as arrays (None when there are none) and start over."""
        if not self.rows:
            return None
        rows, self.rows = self.rows, []
        out = {'stamp': np.array([r['stamp'] for r in rows], dtype=np.float64)}
        for name in self.scalars:
            out[name] = np.array([r[name] for r in rows])
        for name in self.arrays:
            parts = [np.asarray(r[name]).ravel() for r in rows]
            sizes = np.array([0] + [len(p) for p in parts], dtype=np.int64)
            out[name] = np.concatenate(parts) if parts else np.empty(0)
            out[name + '_offsets'] = np.cumsum(sizes)
        return out


def write_arrays(path: str, arrays: dict):
    """Write arrays to path (.npz, compressed), atomically."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp-', suffix='.npz', dir=directory)
    try:
        with os.fdopen(fd, 'wb') as f:
            np.savez_compressed(f, **arrays)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp)
        raise


def row(arrays: dict, name: str, i: int) -> np.ndarray:
    """Return row i of a variable-length field of a chunk read back with np.load."""
    offsets = arrays[name + '_offsets']
    return arrays[name][offsets[i] : offsets[i + 1]]
