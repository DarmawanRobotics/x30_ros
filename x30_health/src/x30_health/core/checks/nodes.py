"""Expected nodes by fully qualified name: '/ns/name' (#13: namespaces count)."""

from __future__ import annotations

from collections import Counter

from x30_health.core.checks.base import Check, Issue, Level
from x30_health.core.config import NodesConfig


def qualified(name: str, namespace: str) -> str:
    """Return the fully qualified node name of (name, namespace)."""
    namespace = '/' + namespace.strip('/')
    return f'{namespace.rstrip("/")}/{name}'


class NodesCheck(Check):
    """One finding per expected node; missing is an error once the start-up grace is over."""

    def __init__(self, config: NodesConfig, grace_until: float):
        self.config = config
        self.grace_until = grace_until
        self.running: Counter = Counter()

    def configure(self, config: NodesConfig):
        """Apply live changes."""
        self.config = config

    def update(self, names_and_namespaces):
        """Store the running nodes as (name, namespace) pairs (get_node_names_and_namespaces)."""
        self.running = Counter(qualified(n, ns) for n, ns in names_and_namespaces)

    def run(self, now: float) -> list[Issue]:
        """Return one finding per expected node."""
        return [self._check(name, now) for name in self.config.expected]

    def _check(self, name: str, now: float) -> Issue:
        key = f'node:{name}'
        count = self.running.get(name, 0)
        if count == 1:
            return Issue(Level.OK, key, f'{name} running')
        if count > 1:
            return Issue(Level.WARN, key, f'{name} runs {count} times', {'count': count})
        if now < self.grace_until:
            return Issue(Level.WARN, key, f'{name} not running yet (start-up)')
        return Issue(Level.ERROR, key, f'{name} not running')
