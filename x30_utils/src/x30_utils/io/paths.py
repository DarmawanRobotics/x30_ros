"""Paths from parameters and services: expanded, and kept inside an allowed root."""

from __future__ import annotations

import os
import re

NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,47}$')


class PathError(ValueError):
    """A path is not allowed."""


def expand(path: str) -> str:
    """Expand ~ and environment variables; '' stays ''."""
    if not path:
        return ''
    return os.path.expandvars(os.path.expanduser(path))


def valid_name(name: str) -> bool:
    """Return True for a site, route or file stem name: 1-48 of a-z, 0-9, _ and -."""
    return bool(NAME_RE.match(name or ''))


def resolve_inside(root: str, path: str) -> str:
    """Return path as an absolute path inside root, relative paths taken from root."""
    root = os.path.realpath(expand(root))
    candidate = expand(path)
    if not os.path.isabs(candidate):
        candidate = os.path.join(root, candidate)
    candidate = os.path.realpath(candidate)
    if candidate != root and not candidate.startswith(root + os.sep):
        raise PathError(f'{path} is outside {root}')
    return candidate
