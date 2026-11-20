"""The acoustic camera's data while recording: spectra and raw frames in chunks, detections."""

from __future__ import annotations

import json
import os

import numpy as np

from x30_recorder.core.chunks import Chunk, write_arrays
from x30_recorder.core.session import stamp_name
from x30_recorder.utils.convert import as_dict, stamp_of
from x30_utils.ros import EVENTS, RECORDED

SPECTRUM = ('bin_hz', 'freq_down_limit', 'freq_up_limit', 'band_peak', 'band_peak_hz')


class AcousticInput:
    """hertzino_driver's spectrum, prpd, voiceprint and detection, written to a session."""

    def __init__(self, node, config, writer):
        self.node, self.config, self.writer = node, config, writer
        self.chunks = {
            'spectrum': Chunk(SPECTRUM, ('magnitude',), config.chunk_sec),
            'prpd': Chunk((), ('data',), config.chunk_sec),
            'voiceprint': Chunk((), ('data',), config.chunk_sec),
        }
        if not config.enabled:
            return
        from hertzino_msgs.msg import DetectionResult, RawFrame, Spectrum

        ns = config.namespace.rstrip('/')
        node.create_subscription(Spectrum, f'{ns}/spectrum', self.on_spectrum, RECORDED)
        for stream in ('prpd', 'voiceprint'):
            node.create_subscription(RawFrame, f'{ns}/{stream}', self.raw(stream), RECORDED)
        node.create_subscription(DetectionResult, f'{ns}/detection', self.on_detection, EVENTS)

    def on_spectrum(self, msg):
        """Keep a spectrum while recording."""
        if self.node.session is not None:
            row = {name: getattr(msg, name) for name in SPECTRUM}
            magnitude = np.asarray(msg.magnitude, dtype=np.int16)
            self.chunks['spectrum'].add(stamp_of(msg, self.node.now()), magnitude=magnitude, **row)

    def raw(self, stream: str):
        """Return the callback keeping a raw frame (PRPD or voiceprint) while recording."""

        def keep(msg):
            if self.node.session is not None:
                data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                self.chunks[stream].add(stamp_of(msg, self.node.now()), data=data)

        return keep

    def on_detection(self, msg):
        """Append a detection result to the session's detections.jsonl."""
        session = self.node.session
        if session is None:
            return
        line = json.dumps(as_dict(msg), sort_keys=True, default=str)
        path = session.file('acoustic', 'detections.jsonl')
        self.writer.put('a detection', lambda: _append(path, line))

    def flush(self, now: float, force: bool = False):
        """Write the chunks that span chunk_sec (all of them with force) into the session."""
        session = self.node.session
        if session is None:
            return
        for name, chunk in self.chunks.items():
            if not (force or chunk.due(now)):
                continue
            arrays = chunk.take()
            if arrays is None:
                continue
            path = session.file('acoustic', f'{name}_{stamp_name(float(arrays["stamp"][0]))}.npz')
            if self.writer.put(f'the {name} chunk', lambda p=path, a=arrays: write_arrays(p, a)):
                session.count('acoustic_chunks')

    def drop(self):
        """Forget what was gathered (a session ended without it)."""
        for chunk in self.chunks.values():
            chunk.take()


def _append(path: str, line: str):
    with open(path, 'a', encoding='utf-8') as f:
        f.write(line + '\n')
        f.flush()
        os.fsync(f.fileno())
