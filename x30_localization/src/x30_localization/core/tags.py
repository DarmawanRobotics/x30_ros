"""Tag files (map poses of fiducials) and the timer-driven session that records them."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os

import numpy as np

from x30_utils.io import read_tags, write_tags
from x30_utils.math import (
    circular_std,
    make_transform,
    quat_from_matrix,
    quat_normalize,
    rpy_from_quat,
    transform_parts,
    yaw_of,
)


def load_tags(path: str) -> tuple[dict[str, np.ndarray], list[str]]:
    """Return ({frame: 4x4 map->tag}, problems); a missing, empty path or broken file is {}."""
    if not path:
        return {}, []
    tags, problems = read_tags(path)
    return {f: make_transform(p[:3], p[3:]) for f, p in tags.items()}, problems


def merge_tags(path: str, new: dict[str, np.ndarray]) -> tuple[dict[str, np.ndarray], str]:
    """Merge new tags into the file (others kept, .bak written); return (all tags, error)."""
    if os.path.exists(path):
        old, problems = load_tags(path)
        if problems:
            return {}, 'tags file left untouched, fix it first: ' + '; '.join(problems)
    else:
        old = {}
    merged = {**old, **new}
    try:
        write_tags(path, {frame: pose_values(m) for frame, m in merged.items()})
    except (OSError, ValueError) as error:
        return {}, f'cannot write {path}: {error}'
    return merged, ''


def pose_values(m: np.ndarray) -> tuple[float, ...]:
    """Return (x, y, z, qx, qy, qz, qw) of a 4x4 pose."""
    xyz, q = transform_parts(m)
    return (*xyz, *(float(v) for v in q))


def describe(m: np.ndarray) -> str:
    """Return 'xyz=(...) rpy=(...)' in metres and degrees."""
    xyz, q = transform_parts(m)
    rpy = (math.degrees(a) for a in rpy_from_quat(*q))
    return 'xyz=({:.3f}, {:.3f}, {:.3f}) rpy=({:.1f}, {:.1f}, {:.1f})'.format(*xyz, *rpy)


@dataclass(frozen=True)
class TagAverage:
    """Mean map pose of a tag over a recording and its spread."""

    pose: np.ndarray
    samples: int
    std_m: float
    std_yaw: float


def average(samples: list[np.ndarray]) -> TagAverage:
    """Average 4x4 poses: mean position, normalized mean quaternion (sign-aligned)."""
    positions = np.array([m[:3, 3] for m in samples])
    quats = np.array([quat_from_matrix(m) for m in samples])
    quats[quats @ quats[0] < 0.0] *= -1.0
    q = quat_normalize(tuple(quats.mean(axis=0)))
    yaws = [yaw_of(m) for m in samples]
    pose = make_transform(positions.mean(axis=0), q)
    std_m = float(positions.std(axis=0).max())
    return TagAverage(pose, len(samples), std_m, circular_std(yaws))


def recordable(localized: bool, contributors: set[str], frames) -> tuple[tuple[str, ...], str]:
    """Return (frames that may be recorded, reason when none may): never a tag from itself."""
    if not localized:
        return (), 'not localized: localize from a tag, a pose estimate or assume_origin first'
    allowed = tuple(f for f in frames if f'tag:{f}' not in contributors)
    if not allowed:
        circular = ', '.join(frames) or 'none configured'
        return (), f'circular: map->odom comes from these tags ({circular}), localize otherwise'
    return allowed, ''


@dataclass(frozen=True)
class RecordLimits:
    """When a recorded tag is good enough to be saved."""

    min_samples: int = 5
    max_std_m: float = 0.05
    max_std_yaw: float = math.radians(3.0)


@dataclass
class RecordSession:
    """Collects map->tag samples per frame until the session ends (no sleeping anywhere)."""

    frames: tuple[str, ...]
    end_time: float
    limits: RecordLimits = field(default_factory=RecordLimits)
    samples: dict[str, list[np.ndarray]] = field(default_factory=dict)
    last_stamp: dict[str, float] = field(default_factory=dict)

    def add(self, frame: str, map_tag: np.ndarray, stamp: float) -> bool:
        """Add one detection (each detection stamp counts once); return True when used."""
        if frame not in self.frames or stamp <= self.last_stamp.get(frame, -math.inf):
            return False
        if not np.all(np.isfinite(map_tag)):
            return False
        self.last_stamp[frame] = stamp
        self.samples.setdefault(frame, []).append(np.array(map_tag, dtype=float))
        return True

    def done(self, now: float) -> bool:
        """Return True once the recording time is over."""
        return now >= self.end_time

    def result(self) -> tuple[dict[str, np.ndarray], list[str]]:
        """Return (good tags, one line per frame) at the end of the session."""
        good, lines = {}, []
        for frame in self.frames:
            values = self.samples.get(frame, [])
            if len(values) < self.limits.min_samples:
                lines.append(
                    f'{frame}: seen {len(values)} times (needs {self.limits.min_samples})'
                )
                continue
            avg = average(values)
            spread = (
                f'n={avg.samples} std={avg.std_m * 100:.1f}cm/{math.degrees(avg.std_yaw):.1f}deg'
            )
            if avg.std_m > self.limits.max_std_m or avg.std_yaw > self.limits.max_std_yaw:
                lines.append(f'{frame}: too noisy, not saved ({spread})')
                continue
            good[frame] = avg.pose
            lines.append(f'{frame}: {describe(avg.pose)} {spread}')
        return good, lines
