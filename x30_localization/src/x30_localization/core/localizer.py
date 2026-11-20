"""Localization policy: explicit sets map->odom, automatic goes through the chi-square gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

import numpy as np

from x30_localization.core.config import TagConfig
from x30_localization.core.estimator import MapOdomEstimator, State
from x30_localization.core.motion import MotionCounter
from x30_localization.core.sources.base import Measurement
from x30_localization.core.sources.tag import TagNoise, TagSource, map_tag_from_base
from x30_localization.core.tags import RecordLimits, RecordSession, merge_tags, recordable
from x30_localization.core.timeline import MapOdomTimeline
from x30_utils.math import Pose2D, compose

Samples = List[Tuple[Pose2D, float]]  # a runtime alias: typing generics for Python 3.8


@dataclass(frozen=True)
class Status:
    """What localization/status reports."""

    state: State
    source: str
    sigma_xy: float
    sigma_yaw: float
    since_correction: float
    message: str


def tag_noise(cfg: TagConfig) -> TagNoise:
    """Return the tag noise model of the configuration."""
    return TagNoise(
        sigma_xy_at_1m=cfg.sigma_xy_at_1m,
        sigma_yaw_at_1m=math.radians(cfg.sigma_yaw_deg_at_1m),
        max_distance_m=cfg.max_distance_m,
        max_view_angle=math.radians(cfg.max_view_angle_deg),
    )


def record_limits(cfg: TagConfig) -> RecordLimits:
    """Return the recording limits of the configuration."""
    return RecordLimits(
        cfg.record_min_samples, cfg.record_max_std_m, math.radians(cfg.record_max_std_deg)
    )


class Localizer:
    """The estimator, its odometry input, the tags and the TF timeline of map->odom."""

    def __init__(self, estimator: MapOdomEstimator, tag_config: TagConfig | None = None):
        self.estimator = estimator
        cfg = estimator.config
        self.motion = MotionCounter(cfg.motion_step_m, math.radians(cfg.motion_step_deg))
        self.timeline = MapOdomTimeline()
        self.tag_config = tag_config or TagConfig()
        self.tags: dict[str, np.ndarray] = {}
        self.last_fused: dict[str, float] = {}
        self.session: RecordSession | None = None
        self.session_epoch = 0
        self.epoch = 0
        self.message = 'not localized'

    def configure(self, tag_config: TagConfig):
        """Apply a new tag configuration (live parameters)."""
        self.tag_config = tag_config

    def track(self, odom_base: Pose2D, now: float) -> Samples:
        """Follow odometry (uncertainty grows with motion); return the periodic TF sample."""
        distance, rotation = self.motion.update(odom_base)
        self.estimator.predict(odom_base, distance, rotation)
        return self.tick(now)

    def tick(self, now: float) -> Samples:
        """Return the periodic TF sample of map->odom (identity while not localized)."""
        if self.timeline.current is None:
            self.timeline.current = Pose2D()
        return self.timeline.tick(now)

    def localize(self, measurement: Measurement, odom_base: Pose2D, now: float) -> Samples:
        """Set map->odom from a trusted measurement (no gate)."""
        self.estimator.set(measurement, odom_base)
        self.epoch += 1
        self.message = f'localized from {measurement.source}'
        return self.timeline.change(self.estimator.map_odom, now)

    def correct(self, measurement: Measurement, odom_base: Pose2D, now: float):
        """Fuse a measurement through the gate; return (Update, TF samples)."""
        result = self.estimator.update(measurement, odom_base)
        if not result.accepted:
            self.message = f'{measurement.source} {result.describe()}'
            return result, []
        self.message = f'corrected from {measurement.source}'
        return result, self.timeline.change(self.estimator.map_odom, now)

    def assume_origin(self, now: float) -> Samples:
        """Take odom as the map: map->odom = identity, known as well as the start spot."""
        cfg = self.estimator.config
        xy = max(cfg.origin_sigma_xy, cfg.min_sigma_xy)
        yaw = math.radians(max(cfg.origin_sigma_yaw_deg, cfg.min_sigma_yaw_deg))
        cov = np.diag([xy**2, xy**2, yaw**2])  # a tag seen soon corrects the spot
        odom_base = self.estimator.odom_base
        return self.localize(Measurement(odom_base, cov, now, 'origin'), odom_base, now)

    def reset(self, now: float, reason: str) -> Samples:
        """Forget map->odom (published as identity while not localized)."""
        self.estimator.reset()
        self.motion.reset()
        self.epoch += 1
        self.last_fused.clear()
        self.message = reason
        return self.timeline.change(Pose2D(), now)

    def measure_tag(
        self, frame: str, m_base_cam: np.ndarray, m_cam_tag: np.ndarray, stamp: float
    ) -> Measurement | str:
        """Return the map->base measurement of a known tag in view, or why not."""
        if frame not in self.tags:
            return f'{frame} is not in the tags file'
        return TagSource(tag_noise(self.tag_config)).measure(
            frame, self.tags[frame], m_base_cam, m_cam_tag, stamp
        )

    def on_tag(self, measurement: Measurement, odom_base: Pose2D, now: float) -> Samples:
        """Use a tag in view automatically: localize while NONE, else correct through the gate."""
        frame = measurement.source.split(':', 1)[-1]
        if self.session is not None and frame in self.session.frames:
            return []  # never correct from a tag while recording it
        last = self.last_fused.get(frame, -math.inf)
        if measurement.stamp - last < max(self.tag_config.min_interval_sec, 1e-9):
            return []
        self.last_fused[frame] = measurement.stamp
        if not self.estimator.localized:
            if not self.tag_config.auto_localize:
                return []
            return self.localize(measurement, odom_base, now)
        if not self.tag_config.auto_correct:
            return []
        return self.correct(measurement, odom_base, now)[1]

    def start_record(self, frames, now: float) -> str:
        """Start recording the map pose of tags; return the reason when refused."""
        if self.session is not None:
            return 'already recording'
        allowed, reason = recordable(
            self.estimator.state is State.LOCALIZED, self.estimator.contributors, frames
        )
        if reason:
            return reason
        end = now + self.tag_config.record_duration_sec
        self.session = RecordSession(allowed, end, record_limits(self.tag_config))
        self.session_epoch = self.epoch
        return ''

    def record_sample(
        self,
        frame: str,
        m_odom_base: np.ndarray,
        m_base_cam: np.ndarray,
        m_cam_tag: np.ndarray,
        stamp: float,
    ):
        """Add the map pose of a detection to the running recording (full 3D chain)."""
        if self.session is None or not self.estimator.localized:
            return
        if np.linalg.norm(m_cam_tag[:3, 3]) > self.tag_config.max_distance_m:
            return  # far detections are too noisy to become site anchors
        m_map_base = compose(self.estimator.map_odom.to_matrix(), m_odom_base)
        self.session.add(frame, map_tag_from_base(m_map_base, m_base_cam, m_cam_tag), stamp)

    def finish_record(self, path: str) -> tuple[bool, str]:
        """End the recording; merge the good tags into path; return (success, message)."""
        session, self.session = self.session, None
        if session is None:
            return False, 'not recording'
        if self.epoch != self.session_epoch or not self.estimator.localized:
            return False, 'localization changed while recording, nothing saved'
        good, lines = session.result()
        if not good:
            return False, 'no tag recorded: ' + '; '.join(lines)
        if not path:
            return False, 'no tags file active: call localization/load_tags first'
        merged, error = merge_tags(path, good)
        if error:
            return False, error
        self.tags = merged
        return True, f'saved {len(good)} tag(s) to {path}\n' + '\n'.join(lines)

    def status(self, now: float) -> Status:
        """Return the current status."""
        est = self.estimator
        if not est.localized:
            return Status(State.NONE, '', math.inf, math.inf, math.inf, self.message)
        since = max(0.0, now - est.corrected_at)
        return Status(est.state, est.source, est.sigma_xy, est.sigma_yaw, since, self.message)
