"""Service calls that never block a callback (x30_utils' AsyncCall) and Trigger hooks."""

from __future__ import annotations

from collections.abc import Callable

from std_srvs.srv import Trigger, TriggerRequest

from x30_utils.ros.services import AsyncCall, ServiceClient

__all__ = ['AsyncCall', 'ServiceClient', 'TriggerHooks', 'call_and_wait']


class TriggerHooks:
    """Named Trigger services called together; callback(ok, messages) when all answered."""

    def __init__(self, node, names: tuple[str, ...]):
        self.node = node
        self.clients = [ServiceClient(Trigger, n) for n in names if n]

    def call_all(self, timeout_sec: float, callback: Callable):
        """Call every hook; callback(all ok, [(name, ok, message)]) once all are done."""
        if not self.clients:
            callback(True, [])
            return
        results = []

        def answered(name):
            def on_result(ok, message):
                results.append((name, ok, message))
                if len(results) == len(self.clients):
                    callback(all(r[1] for r in results), results)

            return on_result

        for client in self.clients:
            AsyncCall(self.node, client, TriggerRequest(), timeout_sec, answered(client.name))


def call_and_wait(client: ServiceClient, request, timeout_sec: float):
    """Call a service and wait: (response or None, error)."""
    return client.call(request, timeout_sec)
