"""Operator source: an RViz 2D Pose Estimate, trusted and applied without the gate."""

from __future__ import annotations

import math

import numpy as np

from x30_localization.core.sources.base import Measurement, Source
from x30_utils.math import Pose2D

TRUSTED_SIGMA_XY = 0.05
TRUSTED_SIGMA_YAW = math.radians(2.0)


class PoseEstimateSource(Source):
    """Turns an operator pose (and its covariance, when usable) into a measurement."""

    name = 'pose_estimate'

    def measure(self, pose: Pose2D, covariance=None, stamp: float = 0.0) -> Measurement | str:
        """Return the measurement: trusted sigmas, or the given covariance when tighter."""
        if not pose.is_finite():
            return 'pose estimate is not finite'
        trusted = np.diag([TRUSTED_SIGMA_XY**2, TRUSTED_SIGMA_XY**2, TRUSTED_SIGMA_YAW**2])
        cov = trusted
        if covariance is not None:
            given = np.asarray(covariance, dtype=float).reshape(3, 3)
            usable = np.all(np.isfinite(given)) and np.all(np.linalg.eigvalsh(given) > 1e-12)
            if usable and np.all(np.diag(given) <= np.diag(trusted)):
                cov = given
        return Measurement(pose, cov, stamp, self.name)
