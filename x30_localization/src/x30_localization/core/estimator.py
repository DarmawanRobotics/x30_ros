"""SE(2) EKF over map->odom; innovations on map->base, uncertainty grows with motion only."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from x30_localization.core.config import EstimatorConfig
from x30_localization.core.sources.base import Measurement
from x30_utils.math import Pose2D, angle_diff, chi2_threshold, wrap_angle

DOF = 3


class State(Enum):
    """Localization state (the values of LocalizationStatus)."""

    NONE = 0
    LOCALIZED = 1
    UNCERTAIN = 2


@dataclass(frozen=True)
class Update:
    """Outcome of a measurement update: squared Mahalanobis distance against the gate."""

    accepted: bool
    distance2: float
    gate: float
    innovation: Pose2D

    def describe(self) -> str:
        """Return a short text for logs."""
        verdict = 'accepted' if self.accepted else 'rejected'
        nu = self.innovation
        return (
            f'{verdict}: d2={self.distance2:.1f} (gate {self.gate:.1f}), robot moved '
            f'{math.hypot(nu.x, nu.y):.2f} m / {math.degrees(nu.yaw):.1f} deg'
        )


def jacobian(map_odom: Pose2D, odom_base: Pose2D) -> np.ndarray:
    """Return H = d(map->base) / d(map->odom) for the additive (x, y, yaw) state."""
    base = map_odom @ odom_base
    arm_x, arm_y = base.x - map_odom.x, base.y - map_odom.y
    return np.array([[1.0, 0.0, -arm_y], [0.0, 1.0, arm_x], [0.0, 0.0, 1.0]])


def base_to_state(map_odom: Pose2D, odom_base: Pose2D, cov_base: np.ndarray) -> np.ndarray:
    """Map a covariance of map->base onto the map->odom state (inverse of H P H^T)."""
    h_inv = np.linalg.inv(jacobian(map_odom, odom_base))
    return h_inv @ np.asarray(cov_base, dtype=float) @ h_inv.T


def symmetric(m: np.ndarray) -> np.ndarray:
    """Return the symmetric part of a matrix (removes rounding asymmetry)."""
    return 0.5 * (m + m.T)


class MapOdomEstimator:
    """Kalman filter of map->odom, fed by odometry motion and map->base measurements."""

    def __init__(self, config: EstimatorConfig | None = None):
        self.config = config or EstimatorConfig()
        self.map_odom: Pose2D | None = None
        self.covariance: np.ndarray | None = None
        self.odom_base = Pose2D()
        self.source = ''
        self.corrected_at = 0.0
        self.contributors: set[str] = set()

    @property
    def localized(self) -> bool:
        """True once map->odom is known."""
        return self.map_odom is not None

    @property
    def state(self) -> State:
        """NONE, LOCALIZED, or UNCERTAIN when the uncertainty is past the limits."""
        if not self.localized:
            return State.NONE
        max_yaw = math.radians(self.config.max_sigma_yaw_deg)
        if self.sigma_xy > self.config.max_sigma_xy or self.sigma_yaw > max_yaw:
            return State.UNCERTAIN
        return State.LOCALIZED

    def reset(self):
        """Forget map->odom."""
        self.map_odom = None
        self.covariance = None
        self.source = ''
        self.contributors = set()

    def set(self, measurement: Measurement, odom_base: Pose2D):
        """Place the robot on the measured map->base, without gate (explicit localization)."""
        self.map_odom = measurement.z @ odom_base.inverse()
        self.covariance = base_to_state(self.map_odom, odom_base, measurement.covariance)
        self._apply_floor()
        self.source = measurement.source
        self.corrected_at = measurement.stamp
        self.contributors = {measurement.source}

    def predict(self, odom_base: Pose2D, distance: float = 0.0, rotation: float = 0.0):
        """Track odometry; grow the uncertainty by the drift of distance (m) and rotation (rad)."""
        self.odom_base = odom_base
        if not self.localized or (distance == 0.0 and rotation == 0.0):
            return
        cfg = self.config
        q_xy = cfg.sigma_xy_per_sqrt_m**2 * abs(distance)
        q_yaw = math.radians(cfg.sigma_yaw_deg_per_sqrt_m) ** 2 * abs(distance)
        q_yaw += math.radians(cfg.sigma_yaw_deg_per_sqrt_rad) ** 2 * abs(rotation)
        q = base_to_state(self.map_odom, odom_base, np.diag([q_xy, q_xy, q_yaw]))
        self.covariance = symmetric(self.covariance + q)

    def gate(self) -> float:
        """Return the chi-square threshold of the innovation (3 dof)."""
        return chi2_threshold(DOF, self.config.gate_confidence)

    def update(self, measurement: Measurement, odom_base: Pose2D, gated: bool = True) -> Update:
        """Fuse map->base measured while odometry read odom_base; outliers fail the gate."""
        if not self.localized:
            raise RuntimeError('update before map->odom is known: use set()')
        predicted = self.map_odom @ odom_base
        z = measurement.z
        nu = np.array([z.x - predicted.x, z.y - predicted.y, angle_diff(z.yaw, predicted.yaw)])
        h = jacobian(self.map_odom, odom_base)
        p, r = self.covariance, measurement.covariance
        s = h @ p @ h.T + r
        distance2 = float(nu @ np.linalg.solve(s, nu))
        result = Update(
            distance2 <= self.gate() or not gated,
            distance2,
            self.gate() if gated else math.inf,
            Pose2D(float(nu[0]), float(nu[1]), float(nu[2])),
        )
        if not result.accepted:
            return result
        k = p @ h.T @ np.linalg.inv(s)
        x = self.map_odom.to_array() + k @ nu
        self.map_odom = Pose2D(float(x[0]), float(x[1]), wrap_angle(float(x[2])))
        i_kh = np.eye(DOF) - k @ h
        self.covariance = symmetric(i_kh @ p @ i_kh.T + k @ r @ k.T)
        self._apply_floor()
        self.source = measurement.source
        self.corrected_at = measurement.stamp
        self.contributors.add(measurement.source)
        return result

    def base_pose(self) -> Pose2D | None:
        """Return map->base at the latest odometry, or None."""
        return None if self.map_odom is None else self.map_odom @ self.odom_base

    def base_covariance(self) -> np.ndarray | None:
        """Return the 3x3 covariance of map->base at the latest odometry, or None."""
        if self.covariance is None:
            return None
        h = jacobian(self.map_odom, self.odom_base)
        return symmetric(h @ self.covariance @ h.T)

    @property
    def sigma_xy(self) -> float:
        """1-sigma position uncertainty of the robot along its largest axis (m)."""
        cov = self.base_covariance()
        if cov is None:
            return math.inf
        return math.sqrt(max(0.0, float(np.linalg.eigvalsh(cov[:2, :2])[-1])))

    @property
    def sigma_yaw(self) -> float:
        """1-sigma heading uncertainty of the robot (rad)."""
        cov = self.base_covariance()
        return math.inf if cov is None else math.sqrt(max(0.0, float(cov[2, 2])))

    def _apply_floor(self):
        """Keep the robot pose covariance above the configured minimum (no overconfidence)."""
        cfg = self.config
        cov = self.base_covariance()
        floor = np.array(
            [cfg.min_sigma_xy**2, cfg.min_sigma_xy**2, math.radians(cfg.min_sigma_yaw_deg) ** 2]
        )
        cov = cov + np.diag(np.maximum(0.0, floor - np.diag(cov)))
        self.covariance = symmetric(base_to_state(self.map_odom, self.odom_base, cov))
