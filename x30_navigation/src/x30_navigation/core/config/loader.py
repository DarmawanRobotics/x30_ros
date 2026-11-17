"""Configuration of path_loader_node."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

MAX_POINTS = 20000


@dataclass(frozen=True)
class ProcessingConfig:
    """Resampling and smoothing of taught routes (stop points never move)."""

    spacing: float = param(0.1, min=0.02, max=1.0, unit='m', doc='point spacing after resampling')
    smooth_weight_data: float = param(0.05, min=0.0, max=1.0, doc='pull toward the recording')
    smooth_weight_smooth: float = param(0.4, min=0.0, max=0.49, doc='pull toward neighbours')
    max_deviation: float = param(
        0.3, min=0.0, max=5.0, unit='m', doc='warn when smoothing moves the route further'
    )


@dataclass(frozen=True)
class LoaderConfig:
    """path_loader_node parameters."""

    sites_dir: str = param(
        '~/x30/map', live=False, doc='routes are loaded from inside this directory only'
    )
    route: str = param('', live=False, doc='route CSV loaded at start (inside sites_dir)')
    remember_file: str = param(
        '~/.x30/last_route', live=False, doc='last loaded route, reloaded at start ("": off)'
    )
    process: bool = param(True, doc='resample and smooth routes between stop points')
    processing: ProcessingConfig = nested(ProcessingConfig)
    republish_sec: float = param(
        2.0, min=0.0, max=60.0, unit='s', doc='navigation/path republish for RViz (0: off)'
    )
