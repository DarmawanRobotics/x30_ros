"""The disk writes of a recording on their own thread: a slow disk never holds a callback."""

# Callbacks hand jobs (a function writing one file) to a bounded queue; one thread runs them in
# order. With the queue full the job is dropped and counted, as a recorder drops frames rather
# than lag: ROS keeps its callbacks, the newest data keeps coming.

from __future__ import annotations

import queue
import threading


class Writer:
    """Run file writes in order on one thread; drop (and count) what does not fit the queue."""

    def __init__(self, size: int, on_error):
        self.jobs: queue.Queue = queue.Queue(maxsize=size)
        self.on_error = on_error  # on_error(what, error), called on the writer's thread
        self.dropped = 0
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, name='recorder-writer', daemon=True)
        self.thread.start()

    def put(self, what: str, job) -> bool:
        """Queue job(); False (and counted) when the queue is full."""
        try:
            self.jobs.put_nowait((what, job))
        except queue.Full:
            with self.lock:
                self.dropped += 1
            return False
        return True

    def drain(self, timeout: float = 10.0) -> bool:
        """Wait until every queued job ran (or timeout); return True when they all did."""
        done = threading.Event()
        if not self.put('drain', done.set):
            return False
        return done.wait(timeout)

    def close(self, timeout: float = 10.0):
        """Run what is queued, then stop the thread."""
        self.drain(timeout)
        self.jobs.put(None)
        self.thread.join(timeout)

    def _run(self):
        while True:
            item = self.jobs.get()
            if item is None:
                return
            what, job = item
            try:
                job()
            except Exception as error:  # one file never stops the recorder
                self.on_error(what, error)
