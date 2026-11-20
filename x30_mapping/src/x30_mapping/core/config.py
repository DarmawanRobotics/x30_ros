"""Configuration of site_manager_node and route_recorder_node."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import param

SITES_DIR = '~/x30/map'


@dataclass(frozen=True)
class SiteConfig:
    """site_manager_node."""

    sites_dir: str = param(SITES_DIR, live=False, doc='one directory per site')
    site: str = param('', live=False, doc='site loaded at start; empty: the one of the last run')
    state_file: str = param('~/.x30/site', live=False, doc='remembers the active site')
    create_site: bool = param(
        True, live=False, doc='create `site` at start when missing: its routes and tags'
    )
    service_timeout_sec: float = param(
        5.0, min=0.5, max=60.0, unit='s', live=False, doc='waiting for localization'
    )
    refresh_sec: float = param(
        2.0, min=0.5, max=60.0, unit='s', live=False, doc='site/status refresh (tags, routes)'
    )


@dataclass(frozen=True)
class RecorderConfig:
    """route_recorder_node."""

    sites_dir: str = param(SITES_DIR, live=False, doc='routes go to <sites_dir>/<site>/routes')
    output_path: str = param(
        '', live=False, doc='directory for routes while no site is active; empty: refuse'
    )
    min_spacing_m: float = param(
        0.2, min=0.01, max=10.0, unit='m', doc='a new waypoint after this much motion'
    )
    record_rate_hz: float = param(
        10.0, min=0.5, max=100.0, unit='Hz', live=False, doc='pose sampling while recording'
    )
    flush_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', doc='the route file is rewritten at most this often'
    )
    max_tf_age_sec: float = param(
        0.5, min=0.01, max=10.0, unit='s', doc='older map->base: no sample'
    )
