from __future__ import annotations

from x30_localization.core.sources.base import Measurement, Source
from x30_localization.core.sources.pose_estimate import PoseEstimateSource
from x30_localization.core.sources.tag import TagSource
from x30_utils.plugins import Registry

sources: Registry[Source] = Registry('x30_localization.sources', Source)
sources.register('tag')(TagSource)
sources.register('pose_estimate')(PoseEstimateSource)

__all__ = [
    'Measurement',
    'PoseEstimateSource',
    'Source',
    'TagSource',
    'sources',
]
