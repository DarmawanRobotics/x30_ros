"""The follower's Trigger hooks: start (waited for, asynchronously), stop and inspections."""

from __future__ import annotations

from std_srvs.srv import TriggerRequest

from x30_navigation.utils.clients import AsyncCall, TriggerHooks


class FollowerHooks:
    """Start, stop and inspection hooks of path_follower_node, never blocking a callback."""

    def __init__(self, node, config, lock):
        self.node = node
        self.lock = lock
        self.config = config
        self.start = TriggerHooks(node, config.start)
        self.stop = TriggerHooks(node, config.stop)
        self.inspection = TriggerHooks(node, config.inspection)
        self.starting = 0

    @property
    def has_start(self) -> bool:
        """True when start hooks must succeed before the robot moves."""
        return bool(self.start.clients)

    def run_start(self, then):
        """Call the start hooks; then(ok, message) once they answered (latest start wins)."""
        self.starting += 1
        ticket = self.starting

        def done(ok, results):
            failed = [f'{name}: {message}' for name, good, message in results if not good]
            with self.lock:
                if ticket == self.starting:
                    then(ok, '; '.join(failed) or 'start hooks done')

        self.start.call_all(self.config.timeout_sec, done)

    def cancel_start(self):
        """Forget a start still waiting for its hooks."""
        self.starting += 1

    def run_stop(self):
        """Call the stop hooks (fire and forget, failures logged)."""

        def done(ok, results):
            if not ok:
                self.node.log.warning('stop hooks failed', results=str(results))

        self.stop.call_all(self.config.timeout_sec, done)

    def run_inspection(self, follower, round_id: int, publish):
        """Call every inspection hook for a stop point round; publish(events) as they answer."""
        ready = [c for c in self.inspection.clients if c.ready()]
        missing = [c.name for c in self.inspection.clients if c not in ready]
        publish(follower.inspection_started(round_id, len(ready), missing))
        for client in ready:
            self._inspect(follower, round_id, client, publish)

    def _inspect(self, follower, round_id, client, publish):
        def answered(ok, message):
            with self.lock:
                publish(follower.inspection_answer(round_id, client.name, ok, message))

        timeout = self.config.inspection_timeout_sec
        AsyncCall(self.node, client, TriggerRequest(), timeout, answered)
