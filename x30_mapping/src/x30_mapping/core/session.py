"""Mapping sessions: which site a grid belongs to and how mapping/start reaches its frame."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

SITE_FRAME_SOURCES = ('pose_estimate',)
SITE_FRAME_PREFIXES = ('tag:',)


class LocState(Enum):
    """Localization state (values of LocalizationStatus.state)."""

    NONE = 0
    LOCALIZED = 1
    UNCERTAIN = 2


@dataclass(frozen=True)
class Localization:
    """What localization last reported."""

    state: LocState = LocState.NONE
    source: str = ''

    @property
    def localized(self) -> bool:
        """True when map->odom is known (even if uncertain)."""
        return self.state is not LocState.NONE

    @property
    def in_site_frame(self) -> bool:
        """True when localized on the site's own anchors (a tag or an operator estimate)."""
        return self.state is LocState.LOCALIZED and (
            self.source in SITE_FRAME_SOURCES or self.source.startswith(SITE_FRAME_PREFIXES)
        )


@dataclass(frozen=True)
class StartPlan:
    """The steps of mapping/start, in order."""

    archive: bool = False  # reset: move the old map, tags and routes into backup-<time>/
    activate: bool = False  # make the site active (load it, hand its tags to localization)
    reset_localization: bool = False  # forget another site's map->odom
    localize: bool = False  # localization/start on one of the site's tags
    assume_origin: bool = False  # not localized: the current pose is the map origin
    resume: bool = False  # continue the saved map (load it into the mapper)


def plan_start(
    site: str,
    reset: bool,
    active_site: str,
    has_content: bool,
    has_map: bool,
    loc: Localization,
    frame_site: str = '',
) -> StartPlan:
    """Return the steps that map site in its own frame."""
    if reset or not has_content:
        return StartPlan(
            archive=reset and has_content, activate=True, assume_origin=not loc.localized
        )
    own_frame = loc.in_site_frame or (loc.localized and frame_site == site)
    if site == active_site and own_frame:
        return StartPlan(resume=has_map)
    return StartPlan(
        activate=True,
        reset_localization=site != active_site,
        localize=True,
        resume=has_map,
    )


class SessionState(Enum):
    """State of the grid mapper."""

    IDLE = 0  # no grid of any site
    MAPPING = 1
    STOPPED = 2  # the grid of `site` is kept for saving


class MappingSession:
    """The grid mapper's session: the site its grid belongs to."""

    def __init__(self):
        self.site = ''
        self.state = SessionState.IDLE

    @property
    def active(self) -> bool:
        """True while scans are integrated."""
        return self.state is SessionState.MAPPING

    def start(self, site: str, reset: bool) -> str:
        """Start mapping site; return why not ('' when started)."""
        if self.active and site != self.site:
            return f'already mapping {self.site}: stop first'
        if not reset and self.site != site:
            held = self.site or 'no site'
            return f'the grid holds {held}: load {site} first, or start it with reset'
        self.site, self.state = site, SessionState.MAPPING
        return ''

    def load(self, site: str) -> str:
        """Take a site's saved map as the grid; return why not."""
        if self.active:
            return f'mapping {self.site}: stop first'
        self.site, self.state = site, SessionState.STOPPED
        return ''

    def stop(self):
        """Stop integrating; the grid stays with its site."""
        if self.state is SessionState.MAPPING:
            self.state = SessionState.STOPPED

    def check_save(self, site: str) -> str:
        """Return why the grid cannot be saved into site ('' when it can)."""
        if self.state is SessionState.IDLE or not self.site:
            return 'nothing mapped yet'
        if site != self.site:
            return f'the grid belongs to {self.site}, not {site}: refusing to overwrite {site}'
        return ''
