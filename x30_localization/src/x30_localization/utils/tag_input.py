"""Tag detections of the node: fresh camera->tag TFs turned into measurements and samples."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import rospy
from tf2_ros import TransformException

from x30_localization.core.sources.base import Measurement
from x30_utils.math import Pose2D
from x30_utils.ros.convert import transform_to_matrix
from x30_utils.ros.time import to_sec


@dataclass(frozen=True)
class Detection:
    """A fresh camera->tag transform; stamp kept as the message (exact TF lookups)."""

    frame: str
    matrix: np.ndarray
    stamp_msg: object

    @property
    def stamp(self) -> float:
        """Detection time in seconds."""
        return to_sec(self.stamp_msg)


class TagInput:
    """Polls the detector's tag frames under the camera and hands them to the localizer."""

    def __init__(self, node, camera_frame: str):
        self.node = node
        self.camera = camera_frame
        self.waiting: dict[str, Detection] = {}  # per frame: ahead of the odometry at its poll

    def detections(self, frames) -> list[Detection]:
        """Return every fresh detection among frames."""
        max_age = self.node.config.tag.max_age_sec
        now = self.node.now()
        out = []
        for frame in sorted(frames):
            try:
                t = self.node.tf.buffer.lookup_transform(self.camera, frame, rospy.Time(0))
            except TransformException:
                continue
            if now - to_sec(t.header.stamp) <= max_age:
                out.append(Detection(frame, transform_to_matrix(t.transform), t.header.stamp))
        return out

    def _context(self):
        node = self.node
        if node.bridge is None:
            return None
        return node.tf.static(node.base, self.camera)

    def due(self, frames) -> list[Detection]:
        """Return per frame the fresh detection that waits for the odometry, else the newest."""
        max_age = self.node.config.tag.max_age_sec
        now = self.node.now()
        newest = {seen.frame: seen for seen in self.detections(frames)}
        out = []
        for frame in sorted(frames):
            waiting = self.waiting.pop(frame, None)
            if waiting is not None and now - waiting.stamp <= max_age:
                out.append(waiting)
            elif frame in newest:
                out.append(newest[frame])
        return out

    def poll(self):
        """Record and fuse every fresh detection (timer)."""
        node, loc = self.node, self.node.localizer
        base_cam = self._context()
        if base_cam is None:
            return
        session = loc.session
        frames = set(loc.tags) | (set(session.frames) if session is not None else set())
        for seen in self.due(frames):
            frame = seen.frame
            m_odom_base = node.odom_base_matrix(seen.stamp_msg)
            if m_odom_base is None:
                # the odometry is behind the camera (FAST-LIO's TF lags its scans): this
                # detection again at the next poll, not a newer one that is ahead again
                self.waiting[frame] = seen
                continue
            with node.lock:
                if loc.session is not None and frame in loc.session.frames:
                    loc.record_sample(frame, m_odom_base, base_cam, seen.matrix, seen.stamp)
                    continue
                measurement = loc.measure_tag(frame, base_cam, seen.matrix, seen.stamp)
                if not isinstance(measurement, Measurement):
                    node.log.debug('tag not used', reason=measurement)
                    continue
                was = loc.estimator.localized
                samples = loc.on_tag(measurement, Pose2D.from_matrix(m_odom_base), node.now())
                node.send(samples)
            if samples:
                node.log.info('map->odom from a tag', tag=frame, auto_start=not was)
                node.publish_status()
            elif 'rejected' in loc.message:
                node.log.warning(loc.message, throttle=5.0)

    def localize_now(self) -> tuple[bool, str]:
        """Localize explicitly from the most precise known tag in view (no gate)."""
        node, loc = self.node, self.node.localizer
        base_cam = self._context()
        if base_cam is None:
            return False, f'frame bridge or URDF {node.base}->{self.camera} not ready'
        if not loc.tags:
            return False, 'no tags loaded'
        best, reasons = None, []
        for seen in self.detections(loc.tags):
            frame = seen.frame
            m_odom_base = node.odom_base_matrix(seen.stamp_msg)
            if m_odom_base is None:
                reasons.append(f'{frame}: no odometry at the detection time')
                continue
            m = loc.measure_tag(frame, base_cam, seen.matrix, seen.stamp)
            if not isinstance(m, Measurement):
                reasons.append(m)
            elif best is None or m.covariance.trace() < best[0].covariance.trace():
                best = (m, Pose2D.from_matrix(m_odom_base))
        if best is None:
            seen = '; '.join(reasons) or f'none of {sorted(loc.tags)} in view'
            return False, f'no usable tag: {seen}'
        with node.lock:
            node.send(loc.localize(best[0], best[1], node.now()))
        return True, f'map->odom set from {best[0].source}'
