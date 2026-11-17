"""Bind configuration dataclasses to the parameter server: validated at start, live ones polled."""

from __future__ import annotations

import contextlib
from typing import Any, Callable, Generic, TypeVar

import rospy

from x30_utils.config import ConfigError, Entry, build, entries, flatten, values_of

T = TypeVar('T')

ROBOT = 'robot'
"""The shared robot profile: searched up from the node (~robot, <ns>/robot, ..., /robot)."""


class LiveConfig(Generic[T]):
    """The current value of a configuration section; attribute access reads the value."""

    def __init__(self, cls: type, prefix: str, value: T):
        self._cls, self._prefix, self._value = cls, prefix, value
        self._callbacks: list = []

    @property
    def value(self) -> T:
        """The configuration as it is now."""
        return self._value

    def on_change(self, callback):
        """Call callback(old, new) after every accepted change."""
        self._callbacks.append(callback)

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._value, name)

    def _set(self, value: T):
        old, self._value = self._value, value
        for callback in self._callbacks:
            callback(old, value)


def server_name(namespace: str, dotted: str) -> str:
    """Return the parameter server name of a dotted name under namespace."""
    return f'{namespace.rstrip("/")}/{dotted.replace(".", "/")}'


class _Section:
    """One declared configuration with where its values live on the server."""

    def __init__(self, config: LiveConfig, items: list[Entry], namespace: str, frozen: bool):
        self.config, self.items, self.namespace = config, items, namespace
        self.names = {e.name for e in items}
        self.frozen = frozen
        self.live = not frozen and any(e.spec.live for e in items)
        self.seen: dict[str, Any] = {}

    def is_live(self, entry: Entry) -> bool:
        """Return True when a server change of entry applies at once."""
        return not self.frozen and entry.spec.live


class ParamBinder:
    """Reads configuration sections from the server and applies later changes of live fields."""

    def __init__(self, namespace: str, overrides: dict[str, Any] | None = None, log=None):
        self.namespace = namespace.rstrip('/') or '/'
        self.overrides = dict(overrides or {})
        self.log = log
        self.sections: list[_Section] = []
        self._fetch: Callable[[str], Any] = _get_param

    def declare(self, cls: type, prefix: str = '', frozen: bool = False) -> LiveConfig:
        """Read every field of cls (prefix.name) from the server; raise ConfigError if invalid."""
        namespace = self.robot_namespace() if prefix.startswith(f'{ROBOT}.') else self.namespace
        items = entries(cls, prefix)
        values = self._values(namespace, prefix, items)
        config = LiveConfig(cls, prefix, build(cls, values, prefix))
        section = _Section(config, items, namespace, frozen)
        # the node's own values go back to the server: rosparam get shows what it runs with
        section.seen = values if frozen else self._publish(namespace, config)
        self.sections.append(section)
        return config

    def _publish(self, namespace: str, config: LiveConfig) -> dict[str, Any]:
        """Write the effective values of a section to the server; return them as it keeps them."""
        effective = {k: _plain(v) for k, v in values_of(config.value, config._prefix).items()}
        for name, value in effective.items():
            if value is None:
                continue  # XML-RPC has no None
            # no server (tests without ROS), or a value it cannot keep: only the view is missing
            with contextlib.suppress(rospy.ROSException, OSError, TypeError, OverflowError):
                rospy.set_param(server_name(namespace, name), value)
        return effective

    def find_robot(self) -> str | None:
        """Return the namespace holding robot/, searched up from the node's, or None."""
        namespace = self.namespace
        while True:
            if _has_param(server_name(namespace, ROBOT)):
                return namespace
            if namespace in ('', '/'):
                return None
            namespace = namespace.rsplit('/', 1)[0] or '/'

    def robot_namespace(self) -> str:
        """Return the namespace holding robot/ (the node's own when there is none)."""
        return self.find_robot() or self.namespace

    def names(self) -> set[str]:
        """Return every declared parameter name (dotted)."""
        return {n for s in self.sections for n in s.names}

    def unknown(self) -> list[str]:
        """Return the node's parameters that no section declares (typos), dotted."""
        tree = self._fetch(self.namespace)
        known = self.names()
        return sorted(n for n in flatten(tree) if n not in known and not n.startswith(f'{ROBOT}.'))

    def poll(self) -> list[str]:
        """Apply server changes of live fields, refuse the others; one line per change."""
        notes = []
        for section in self.sections:
            values = self._values(section.namespace, section.config._prefix, section.items)
            changed = {k for k in section.names if values.get(k) != section.seen.get(k)}
            if not changed:
                continue
            section.seen = values
            notes.append(self._apply(section, values, changed))
        return notes

    def _apply(self, section: _Section, values: dict[str, Any], changed: set[str]) -> str:
        frozen = sorted(
            e.name for e in section.items if e.name in changed and not section.is_live(e)
        )
        if frozen:
            return f'ignored, read-only until restart: {", ".join(frozen)}'
        merged = {e.name: values.get(e.name, e.default) for e in section.items}
        try:
            new = build(section.config._cls, merged, section.config._prefix)
        except ConfigError as error:
            return f'rejected, kept the old values: {error}'
        section.config._set(new)
        return f'applied: {", ".join(sorted(changed))}'

    def _values(self, namespace: str, prefix: str, items: list[Entry]) -> dict[str, Any]:
        root = prefix.split('.', 1)[0] if prefix else ''
        tree = self._fetch(server_name(namespace, root) if root else namespace)
        flat = flatten(tree, f'{root}.' if root else '')
        values = {e.name: _plain(flat[e.name]) for e in items if e.name in flat}
        names = {e.name for e in items}
        values.update({k: _plain(v) for k, v in self.overrides.items() if k in names})
        return values


def _plain(value: Any) -> Any:
    """Return value as the parameter server keeps it (lists, not tuples)."""
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def _has_param(name: str) -> bool:
    try:
        return rospy.has_param(name)
    except (rospy.ROSException, OSError):
        return False


def _get_param(name: str) -> Any:
    try:
        return rospy.get_param(name, {})
    except (KeyError, rospy.ROSException, OSError):
        return {}
