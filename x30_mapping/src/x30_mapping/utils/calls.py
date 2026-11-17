"""Blocking service calls from a node's callbacks, with a timeout (rospy has none per call)."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any

import rospy

READY_CHECK_SEC = 0.05


@dataclass(frozen=True)
class CallResult:
    """The answer of a service call."""

    response: Any = None
    error: str = ''
    unavailable: bool = False  # the server is not running

    @property
    def ok(self) -> bool:
        """True when the server answered with success (or has no success field)."""
        return self.response is not None and bool(getattr(self.response, 'success', True))

    @property
    def message(self) -> str:
        """The error, or the server's message."""
        return self.error or str(getattr(self.response, 'message', ''))


class Client:
    """A service of another node, by its resolved name."""

    def __init__(self, srv_type, name: str):
        self.proxy = rospy.ServiceProxy(name, srv_type)
        self.name = self.proxy.resolved_name

    def wait(self, timeout_sec: float) -> bool:
        """Wait at most timeout_sec for the server; True once it is there."""
        try:
            self.proxy.wait_for_service(timeout=timeout_sec)
        except rospy.ROSException:  # timed out, or shutting down
            return False
        return True

    def ready(self) -> bool:
        """Return True when the server is there now."""
        return self.wait(READY_CHECK_SEC)


class Caller:
    """Service clients of one node, called with a timeout."""

    def __init__(self, node, timeout_sec: float):
        self.node = node
        self.timeout_sec = timeout_sec

    def client(self, srv_type, name: str) -> Client:
        """Return a client of the service name (resolved in the node's namespace)."""
        return Client(srv_type, name)

    def call(self, client: Client, request, timeout_sec: float | None = None) -> CallResult:
        """Call and wait for the answer at most the timeout (the call itself is not cancelled)."""
        timeout = self.timeout_sec if timeout_sec is None else timeout_sec
        if not client.wait(min(timeout, 2.0)):
            return CallResult(error=f'{client.name} not available', unavailable=True)
        answer: list[CallResult] = []
        done = threading.Event()

        def run():
            try:
                answer.append(CallResult(client.proxy.call(request)))
            except Exception as error:  # a failed call is an answer too
                answer.append(CallResult(error=f'{client.name} failed: {error}'))
            finally:
                done.set()

        threading.Thread(target=run, name=f'call {client.name}', daemon=True).start()
        if not done.wait(timeout):
            return CallResult(error=f'{client.name} timed out after {timeout:.1f} s')
        return answer[0]
