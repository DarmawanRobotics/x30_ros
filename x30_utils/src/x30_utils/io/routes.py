"""Taught routes as CSV: x, y (m, map frame), yaw_deg, dwell_sec (> 0: stop point)."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import io
import math

from x30_utils.io.files import write_text_atomic

HEADER = ('x', 'y', 'yaw_deg', 'dwell_sec')


@dataclass(frozen=True)
class RoutePoint:
    """One waypoint of a route; yaw in radians."""

    x: float
    y: float
    yaw: float = 0.0
    dwell_sec: float = 0.0

    @property
    def is_stop(self) -> bool:
        """True for a stop point."""
        return self.dwell_sec > 0.0


class RouteError(ValueError):
    """The route file cannot be used."""


def read_route(path: str) -> list[RoutePoint]:
    """Read a route CSV (yaw_deg or yaw in radians); raise RouteError on bad rows."""
    try:
        with open(path, encoding='utf-8', newline='') as f:
            rows = list(csv.DictReader(f))
    except OSError as error:
        raise RouteError(f'{path}: {error}') from error
    points = []
    for n, row in enumerate(rows, start=2):
        try:
            yaw = math.radians(float(row['yaw_deg'])) if 'yaw_deg' in row else float(row['yaw'])
            point = RoutePoint(
                float(row['x']), float(row['y']), yaw, float(row.get('dwell_sec') or 0.0)
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RouteError(f'{path}:{n}: {error}') from error
        if not all(math.isfinite(v) for v in (point.x, point.y, point.yaw, point.dwell_sec)):
            raise RouteError(f'{path}:{n}: non-finite value')
        points.append(point)
    return points


def format_route(points: list[RoutePoint]) -> str:
    """Return the CSV text of a route."""
    out = io.StringIO()
    writer = csv.writer(out, lineterminator='\n')
    writer.writerow(HEADER)
    for p in points:
        writer.writerow(
            (f'{p.x:.3f}', f'{p.y:.3f}', f'{math.degrees(p.yaw):.1f}', f'{p.dwell_sec:.1f}')
        )
    return out.getvalue()


def write_route(path: str, points: list[RoutePoint]):
    """Write a route atomically."""
    write_text_atomic(path, format_route(points))
