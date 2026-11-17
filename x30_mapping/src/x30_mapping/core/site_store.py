"""Sites on disk (map, tags, routes, backups), loaded tolerantly: problems, never raises."""

from __future__ import annotations

from dataclasses import dataclass, field
import os
import shutil

from x30_mapping.core.map_io import BAK, MAP_IMAGE, MAP_YAML, GridMap, load_map
from x30_mapping.core.pgm import MapError
from x30_utils.io import PathError, SiteLayout, expand, list_sites, read_tags, unique_dir
from x30_utils.io import valid_name as _valid_name

TAGS_FILE, ROUTES_DIR = 'tags.json', 'routes'
CONTENT = (MAP_YAML, MAP_IMAGE, TAGS_FILE)
ARCHIVE_PREFIX = 'backup'


def valid_name(name: str) -> bool:
    """Return True for a valid site name (1-48 of a-z, 0-9, _ and -)."""
    return isinstance(name, str) and _valid_name(name)


@dataclass
class SiteSummary:
    """What a site holds, as loaded (problems: what could not be read)."""

    site: str
    map: GridMap | None = None
    tags: int = 0
    routes: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)

    @property
    def has_content(self) -> bool:
        """True when the site has a map or fiducials (a frame worth keeping)."""
        return self.map is not None or self.tags > 0


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

    def has_map(self, site: str) -> bool:
        """Return True when the site has a map file (readable or not)."""
        return self.exists(site) and os.path.isfile(self.layout(site).map_yaml)

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

    def load_map(self, site: str) -> tuple[GridMap | None, list[str]]:
        """Return (the site map or None, problems); a broken map falls back to map.yaml.bak."""
        if not self.exists(site):
            return None, []
        layout = self.layout(site)
        problems = []
        for path in (layout.map_yaml, layout.map_yaml + BAK):
            if not os.path.isfile(path):
                continue
            try:
                grid = load_map(path)
            except MapError as error:
                problems.append(f'map not loaded: {error}')
                continue
            if path.endswith(BAK):
                problems.append(f'using the previous map {os.path.basename(path)}')
            return grid, problems
        return None, problems

    def summary(self, site: str) -> SiteSummary:
        """Load everything a site holds, tolerating broken files."""
        grid, problems = self.load_map(site)
        tags, tag_problems = self.tags(site)
        return SiteSummary(site, grid, tags, self.routes(site), problems + tag_problems)

    def has_content(self, site: str) -> bool:
        """Return True when the site holds a map, fiducials (even unreadable) or routes."""
        if not self.exists(site):
            return False
        layout = self.layout(site)
        if any(os.path.isfile(f) for f in (layout.map_yaml, layout.map_yaml + BAK)):
            return True
        tags, problems = self.tags(site)
        return tags > 0 or bool(problems) or bool(self.routes(site))

    def archive(self, site: str) -> str | None:
        """Move map, tags and routes (and their .bak) into a new backup-<time>/; return it."""
        layout = self.layout(site)
        names = [n for name in CONTENT for n in (name, name + BAK)]
        present = [n for n in names if os.path.exists(os.path.join(layout.root, n))]
        if os.path.isdir(layout.routes) and os.listdir(layout.routes):
            present.append(ROUTES_DIR)
        if not present:
            return None
        backup = unique_dir(layout.root, ARCHIVE_PREFIX)
        for name in present:
            shutil.move(os.path.join(layout.root, name), os.path.join(backup, name))
        os.makedirs(layout.routes, exist_ok=True)
        return backup
