"""Named implementations of an interface, built in or added by other packages (entry points)."""

from __future__ import annotations

from importlib import metadata
from typing import Generic, TypeVar

T = TypeVar('T')


class Registry(Generic[T]):
    """Map names to classes implementing one interface."""

    def __init__(self, group: str, base: type[T] | None = None):
        self.group = group
        self.base = base
        self._classes: dict[str, type[T]] = {}
        self._loaded = False

    def register(self, name: str):
        """Class decorator adding an implementation under name."""

        def wrap(cls):
            self._check(name, cls)
            self._classes[name] = cls
            return cls

        return wrap

    def names(self) -> list[str]:
        """Return every available name, sorted."""
        self._discover()
        return sorted(self._classes)

    def get(self, name: str) -> type[T]:
        """Return the class registered under name."""
        self._discover()
        try:
            return self._classes[name]
        except KeyError:
            raise KeyError(f'unknown {self.group} "{name}", use one of {self.names()}') from None

    def create(self, name: str, *args, **kwargs) -> T:
        """Instantiate the implementation registered under name."""
        return self.get(name)(*args, **kwargs)

    def __contains__(self, name: str) -> bool:
        self._discover()
        return name in self._classes

    def _discover(self):
        if self._loaded:
            return
        self._loaded = True
        for entry in entry_points(self.group):
            if entry.name not in self._classes:
                cls = entry.load()
                self._check(entry.name, cls)
                self._classes[entry.name] = cls

    def _check(self, name: str, cls):
        if self.base is not None and not (isinstance(cls, type) and issubclass(cls, self.base)):
            raise TypeError(f'{self.group} "{name}": {cls!r} is not a {self.base.__name__}')


def entry_points(group: str) -> list:
    """Return the installed entry points of a group (Python 3.8 to 3.12+)."""
    try:
        found = metadata.entry_points(group=group)  # py38: ok
    except TypeError:  # Python < 3.10: no selection, a dict of group -> entry points
        return list(metadata.entry_points().get(group, ()))
    return list(found)
