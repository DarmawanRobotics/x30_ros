"""What every localization source produces: a measurement of map->base with its covariance."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from x30_utils.math import Pose2D


@dataclass(frozen=True)
class Measurement:
    """map->base observed at stamp (s); covariance 3x3 over (x, y, yaw) in map axes."""

    z: Pose2D
    covariance: np.ndarray
    stamp: float
    source: str

    def __post_init__(self):
        cov = np.array(self.covariance, dtype=float).reshape(3, 3)
        object.__setattr__(self, 'covariance', 0.5 * (cov + cov.T))

    def valid(self) -> bool:
        """Return True when the pose is finite and the covariance positive definite."""
        cov = self.covariance
        if not self.z.is_finite() or not np.all(np.isfinite(cov)):
            return False
        return bool(np.all(np.linalg.eigvalsh(cov) > 0.0))


class Source(ABC):
    """Turns one kind of observation into Measurements of map->base."""

    name = ''

    @abstractmethod
    def measure(self, *args, **kwargs) -> Measurement | str:
        """Return a measurement, or the reason why there is none."""
