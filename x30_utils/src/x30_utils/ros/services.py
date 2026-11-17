"""Service clients for rospy: readiness, a timeout per call, and calls that never block a node."""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Any

import rospy

READY_CHECK_SEC = 0.05
DEFER_SEC = 0.001  # done of a call that could not start: right after the caller's callback


class ServiceClient:
    """A service of another node, by its resolved name."""

    def __init__(self, srv_type, name: str):
        self.proxy = rospy.ServiceProxy(name, srv_type)
        self.name = self.proxy.resolved_name
        self.request_class = srv_type._request_class

    def wait(self, timeout_sec: float) -> bool:
        """Wait at most timeout_sec for the server; return True once it is there."""
        try:
            self.proxy.wait_for_service(timeout=max(timeout_sec, 1e-3))
        except rospy.ROSException:  # timed out, or shutting down
            return False
        return True

    def ready(self) -> bool:
        """Return True when the server is there now."""
        return self.wait(READY_CHECK_SEC)

    def call(self, request, timeout_sec: float) -> tuple[Any, str]:
        """Call and wait at most timeout_sec: (response, '') or (None, why); never raises."""
        if not self.wait(min(timeout_sec, 2.0)):
            return None, f'{self.name} not available'
        answer: list[tuple[Any, str]] = []
        done = threading.Event()

        def run():
            answer.append(self._call(request))
            done.set()

        threading.Thread(target=run, name=f'call {self.name}', daemon=True).start()
        if not done.wait(timeout_sec):
            return None, f'{self.name} timed out after {timeout_sec:.1f} s'
        return answer[0]

    def _call(self, request) -> tuple[Any, str]:
        try:
            return self.proxy.call(request), ''
        except Exception as error:  # a failed call is an answer too
            return None, f'{self.name} failed: {error}'


class AsyncCall:
    """Call a service in a thread; report once, as one of the node's callbacks: done(ok, msg)."""

    # done never runs inside the constructor: a caller holding its own lock never waits there
    # for the node's (the lock order stays the node's first)

    def __init__(self, node, client: ServiceClient, request, timeout_sec: float, done: Callable):
        self.client = client
        self._done = node.guard(done, f'{client.name} answer failed')
        self._lock = threading.Lock()
        self.finished = False
        self._timer = None
        if not client.ready():
            missing = f'{client.name} not available'
            self._timer = node.after(DEFER_SEC, lambda: self._finish(False, missing))
            return
        self._timer = node.after(timeout_sec, self._on_timeout)
        threading.Thread(target=self._run, args=(request,), name=f'call {client.name}',
                         daemon=True).start()  # fmt: skip

    def _run(self, request):
        response, error = self.client._call(request)
        if error:
            self._finish(False, error)
        elif response is None:
            self._finish(False, f'{self.client.name} gave no answer')
        else:
            ok = bool(getattr(response, 'success', True))
            self._finish(ok, str(getattr(response, 'message', '')))

    def _on_timeout(self):
        self._finish(False, f'{self.client.name} timed out')

    def _finish(self, ok: bool, message: str):
        with self._lock:
            if self.finished:
                return
            self.finished = True
        if self._timer is not None:
            self._timer.shutdown()
        self._done(ok, message)
