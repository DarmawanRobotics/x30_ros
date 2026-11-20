"""One handheld at a time: who drives, when it was last heard, its acknowledgements."""

from __future__ import annotations

import hmac

MAX_ACKS = 200  # acknowledgements kept per session (resent commands are answered from here)
MAX_DENIED = 64  # addresses remembered for the `denied` rate limit


class Session:
    """The current client of the gateway and the rules for taking it over."""

    def __init__(self, token: str, timeout: float):
        self.token = token
        self.timeout = timeout
        self.client = None  # (ip, port)
        self.name = ''
        self.seen = 0.0
        self.number = 0  # increments per new client: late acks of an old session are dropped
        self.acks: dict = {}
        self.denied: dict = {}

    def authorized(self, message: dict) -> bool:
        """Check the shared token (constant time) when one is configured."""
        if not self.token:
            return True
        return hmac.compare_digest(str(message.get('k', '')), self.token)

    def deny_now(self, addr, now: float) -> bool:
        """Return True when a `denied` may go to addr (at most once per second each)."""
        if now - self.denied.get(addr, -1.0) < 1.0:
            return False
        if len(self.denied) > MAX_DENIED:
            self.denied = {a: t for a, t in self.denied.items() if now - t < 1.0}
        self.denied[addr] = now
        return True

    def alive(self, now: float) -> bool:
        """Return True while the client was heard within the timeout."""
        return self.client is not None and now - self.seen <= self.timeout

    def busy_for(self, addr, now: float) -> str:
        """Return why addr may not drive now ('' when it may): another device is driving."""
        current = self.client
        if current is not None and addr != current and addr[0] != current[0] and self.alive(now):
            return f'robot is driven from {current[0]}'
        return ''

    def accept(self, addr, name: str, now: float) -> bool:
        """Make addr the client (the same device may come back from another port); True: new."""
        self.seen = now
        self.name = str(name)[:32]
        if addr == self.client:
            return False
        self.end()
        self.client = addr
        self.number += 1
        return True

    def end(self) -> bool:
        """Forget the client and its acknowledgements; True when there was one."""
        had = self.client is not None
        self.client = None
        self.acks.clear()
        return had

    def remember(self, key: tuple, reply: dict):
        """Keep an acknowledgement for resends of the same (id, name)."""
        self.acks[key] = reply
        if len(self.acks) > MAX_ACKS:
            self.acks.pop(next(iter(self.acks)))
