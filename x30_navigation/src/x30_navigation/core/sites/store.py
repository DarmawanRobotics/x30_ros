"""Sites on disk (tags, routes), loaded tolerantly: problems, never raises."""

from __future__ import annotations

from dataclasses import dataclass, field
import os

from x30_utils.io import PathError, SiteLayout, expand, list_sites, read_tags
from x30_utils.io import valid_name as _valid_name


def valid_name(name: str) -> bool:
    """Return True for a valid site name (1-48 of a-z, 0-9, _ and -)."""
    return isinstance(name, str) and _valid_name(name)


@dataclass
class SiteSummary:
    """What a site holds, as loaded (problems: what could not be read)."""

    site: str
    tags: int = 0
    routes: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)


class SiteStore:
    """The sites under one directory."""

    def __init__(self, sites_dir: str):
        self.root = os.path.abspath(expand(sites_dir))

    def layout(self, site: str) -> SiteLayout:
        """Return the paths of a site; raise PathError for an invalid name."""
        if not valid_name(site):
            raise PathError(
                f'site {site!r} must be 1-48 of a-z, 0-9, _ and -, starting with a letter or digit'
            )
        return SiteLayout(self.root, site)

    def list_sites(self) -> list[str]:
        """Return every site name, sorted."""
        try:
            return list_sites(self.root)
        except OSError:
            return []

    def exists(self, site: str) -> bool:
        """Return True when the site directory exists."""
        return valid_name(site) and os.path.isdir(self.layout(site).root)

    def create(self, site: str) -> str:
        """Create the site directory and routes/ when missing (OSError); return it."""
        layout = self.layout(site)
        os.makedirs(layout.routes, exist_ok=True)
        return layout.root

    def tags(self, site: str) -> tuple[int, list[str]]:
        """Return (number of valid fiducials, problems)."""
        if not self.exists(site):
            return 0, []
        tags, problems = read_tags(self.layout(site).tags)
        return len(tags), problems

    def routes(self, site: str) -> list[str]:
        """Return the route files of a site (names in routes/), sorted."""
        if not self.exists(site):
            return []
        try:
            names = os.listdir(self.layout(site).routes)
        except OSError:
            return []
        return sorted(n for n in names if n.endswith('.csv') and not n.startswith('.'))

    def summary(self, site: str) -> SiteSummary:
        """Load everything a site holds, tolerating broken files."""
        tags, problems = self.tags(site)
        return SiteSummary(site, tags, self.routes(site), problems)
