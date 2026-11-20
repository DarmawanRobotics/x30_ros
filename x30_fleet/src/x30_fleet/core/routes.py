"""Route ids safe as file names, and the summary NETRA keeps of a taught route."""

from __future__ import annotations

import csv
import io
import math
import re

ROUTE_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def safe_route_id(route_id) -> bool:
    """Return True for ids that are safe as a file name (no paths, no dot files)."""
    return isinstance(route_id, (str, int)) and bool(ROUTE_ID_RE.match(str(route_id)))


def summarize_csv(text: str) -> dict:
    """Return length (m), point and stop counts of a recorded route CSV."""
    points, stops = [], 0
    for row in csv.reader(io.StringIO(text)):
        if not row or row[0].strip().startswith('#'):
            continue
        try:
            x, y = float(row[0]), float(row[1])
            dwell = float(row[3]) if len(row) > 3 and row[3].strip() else 0.0
        except (ValueError, IndexError):
            continue
        points.append((x, y))
        stops += dwell > 0.0
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    closed = len(points) > 2 and math.dist(points[0], points[-1]) < 1.0
    return {'length_m': round(length, 2), 'points': len(points), 'stops': stops, 'closed': closed}
