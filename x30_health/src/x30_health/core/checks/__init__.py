"""Health checks: one Check per family; products add their own through the registry."""

from __future__ import annotations

from x30_health.core.checks.base import Check, Issue, Level
from x30_utils.plugins import Registry

# Checks a product adds without forking: an entry point in the group 'x30_health.checks'
# naming a Check subclass that takes no arguments; enabled with the extra_checks parameter.
registry: Registry[Check] = Registry('x30_health.checks', Check)

__all__ = ['Check', 'Issue', 'Level', 'registry']
