"""The writer: jobs in order on one thread, drops counted when full, failures reported."""

from __future__ import annotations

import threading

from x30_recorder.core.writer import Writer


def test_jobs_run_in_order_and_failures_are_reported():
    errors, done = [], []
    writer = Writer(8, lambda what, error: errors.append((what, str(error))))
    for i in range(5):
        assert writer.put(f'job {i}', lambda i=i: done.append(i))
    assert writer.put('bad', lambda: 1 / 0)
    assert writer.drain(5.0)
    assert done == [0, 1, 2, 3, 4] and errors == [('bad', 'division by zero')]
    writer.close()


def test_a_full_queue_drops_rather_than_blocks():
    release = threading.Event()
    writer = Writer(2, lambda *_: None)
    assert writer.put('slow', release.wait)  # holds the thread
    accepted = sum(writer.put(f'job {i}', lambda: None) for i in range(10))
    assert accepted <= 2 and writer.dropped == 10 - accepted
    release.set()
    writer.close()
