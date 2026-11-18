"""Blocking service calls from a node's callbacks, with a timeout (x30_utils' ServiceClient)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from x30_utils.ros.services import ServiceClient


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


class Caller:
    """Service clients of one node, called with a timeout."""

    def __init__(self, node, timeout_sec: float):
        self.node = node
        self.timeout_sec = timeout_sec

    def client(self, srv_type, name: str) -> ServiceClient:
        """Return a client of the service name (resolved in the node's namespace)."""
        return ServiceClient(srv_type, name)

    def call(self, client: ServiceClient, request, timeout_sec: float | None = None) -> CallResult:
        """Call and wait for the answer at most the timeout (the call itself is not cancelled)."""
        timeout = self.timeout_sec if timeout_sec is None else timeout_sec
        if not client.wait(min(timeout, 2.0)):
            return CallResult(error=f'{client.name} not available', unavailable=True)
        response, error = client.call(request, timeout)
        return CallResult(error=error) if error else CallResult(response)
