"""NETRA's substation module and x30_inspection: the plan as theirs, their reports as uploads."""

# NETRA plans the points of a route by stop (the rows of the route CSV with a dwell, from 0): the
# camera pose, the regions of the thermal image (fractions of the frame) or a partial discharge
# detection. x30_inspection inspects points by where the robot stands: each stop becomes a point
# netra_stop<k> at that row, each NETRA point a target netra_<id>, its regions named by their key.
# A report of such a target becomes an upload of that NETRA point.

from __future__ import annotations

import csv
import io
import math
import os
import re
import time

TARGET = re.compile(r'^netra_(\d+)$')
PAN, TILT, ZOOM = (-360.0, 360.0), (-90.0, 90.0), (1.0, 60.0)  # x30_inspection's limits
MAX_LIMIT_C = 500.0  # x30_inspection's highest own limit (NETRA allows up to 1500)
PD_DB = (-20.0, 200.0)  # what NETRA takes of a partial discharge reading
FRAME = {'x': 0.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}  # a NETRA target without a region
KEY = re.compile(r'^[a-z0-9][a-z0-9_-]{0,47}$')
UID_CHARS = re.compile(r'[^A-Za-z0-9._-]')
IMAGES = (('thermal', '_thermal.jpg'), ('visual', '_visible.jpg'), ('acoustic', '_soundmap.jpg'))


def number(value, default: float | None = None) -> float | None:
    """Return a finite int / float as float (bools and text are not numbers), else default."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return float(value)


def route_stops(text: str) -> list[tuple[float, float]]:
    """Return (x, y) of the rows of a route CSV with a dwell: its stops, in order."""
    stops = []
    for row in csv.reader(io.StringIO(text)):
        if not row or row[0].strip().startswith('#'):
            continue
        try:
            x, y = float(row[0]), float(row[1])
            dwell = float(row[3]) if len(row) > 3 and row[3].strip() else 0.0
        except (ValueError, IndexError):
            continue
        if dwell > 0.0:
            stops.append((x, y))
    return stops


def clamp(value, limits: tuple[float, float], default: float) -> float:
    """Return value within limits (default when it is not a number)."""
    v = number(value, default)
    return min(limits[1], max(limits[0], v))


def regions(point: dict) -> list[dict]:
    """Return the x30_inspection regions of a NETRA thermal point (its targets' rois)."""
    out = []
    for target in point.get('targets') or []:
        if not isinstance(target, dict):
            continue
        roi, key = target.get('roi') or FRAME, str(target.get('key', ''))
        values = [number(roi.get(k)) if isinstance(roi, dict) else None for k in 'xywh']
        if not KEY.match(key) or None in values:
            continue
        x, y, w, h = values
        box = [max(0.0, x), max(0.0, y), min(1.0, x + w), min(1.0, y + h)]
        if box[2] > box[0] and box[3] > box[1]:
            out.append({'name': key, 'box': [round(v, 4) for v in box]})
    return out


def target(point: dict) -> dict:
    """Return the x30_inspection target of one NETRA point."""
    kind, ptz = point.get('kind'), point.get('ptz') or {}
    limits = [number(t.get('limit_c')) for t in point.get('targets') or [] if isinstance(t, dict)]
    limits = [v for v in limits if v is not None and v > 0.0]
    return {
        'name': f'netra_{int(point["id"])}',
        'pan_deg': clamp(ptz.get('pan_deg'), PAN, 0.0),
        'tilt_deg': clamp(ptz.get('tilt_deg'), TILT, 0.0),
        'zoom': clamp(ptz.get('zoom'), ZOOM, 1.0),
        'thermal': {
            'enabled': kind == 'thermal',
            'strips': 0,
            'regions': regions(point) if kind == 'thermal' else [],
            'max_temp_c': min(*limits, MAX_LIMIT_C) if limits else 0.0,
        },
        'acoustic': {'enabled': kind == 'pd'},
    }


def to_plan(plan: dict, stops: list[tuple[float, float]]) -> tuple[dict, list[str]]:
    """Return x30_inspection's plan of NETRA's plan of a route, and what was left out."""
    points: dict[int, dict] = {}
    skipped = []
    listed = [p for p in plan.get('points') or [] if isinstance(p, dict)]
    for point in sorted(
        listed, key=lambda p: (number(p.get('stop_index'), -1), number(p.get('seq'), 0))
    ):
        index = number(point.get('stop_index'))
        if number(point.get('id')) is None or point.get('kind') not in ('thermal', 'pd'):
            skipped.append(f'point {point.get("id")!r}: not a thermal or pd point')
            continue
        if index is None or not 0 <= int(index) < len(stops):
            skipped.append(
                f'point {point.get("id")}: stop {point.get("stop_index")} not in the route'
            )
            continue
        k = int(index)
        x, y = stops[k]
        entry = points.setdefault(k, {'name': f'netra_stop{k}', 'x': x, 'y': y, 'targets': []})
        entry['targets'].append(target(point))
    return {'points': [points[k] for k in sorted(points)]}, skipped


def point_id(name: str) -> int | None:
    """Return the NETRA point of an x30_inspection target name (netra_<id>), else None."""
    match = TARGET.match(name or '')
    return int(match.group(1)) if match else None


def uploads(report: dict, directory: str, robot_id: str, patrol_id, yaw: float) -> list:
    """Return [(meta, {field: path})] of a report.json: one upload per NETRA point measured."""
    out = []
    folder = os.path.relpath(directory, os.path.dirname(os.path.dirname(directory)))
    for item in report.get('targets') or []:
        pid = point_id(item.get('target', ''))
        if pid is None or not item.get('measured'):
            continue
        meta = {
            'uid': UID_CHARS.sub('-', f'{robot_id}.{folder}.{pid}')[:128],
            'point_id': pid,
            'ts': number(report.get('stamp'), time.time()),
            'patrol_id': patrol_id,
            'pose': {
                'x': number(report.get('x'), 0.0),
                'y': number(report.get('y'), 0.0),
                'yaw': round(yaw, 3),
            },
        }
        if number(report.get('ambient_c')) is not None:
            meta['ambient_c'] = report['ambient_c']
        readings = [r for r in item.get('readings') or [] if number(r.get('max_c')) is not None]
        if readings:
            meta['thermal'] = [{'key': r['region'], 'max_c': r['max_c'], 'min_c': r.get('min_c')}
                               for r in readings]  # fmt: skip
        sound = next(
            (f for f in item.get('findings') or [] if f.get('sensor') == 'acoustic'), None
        )
        if sound is not None:
            label = str(sound.get('label', ''))
            db = clamp(sound.get('value'), PD_DB, 0.0)
            meta['pd'] = {'db': db, 'detected': label != 'noise', 'kind': label[:32]}
        if 'thermal' not in meta and 'pd' not in meta:
            continue
        files = {}
        for field, suffix in IMAGES:
            path = os.path.join(directory, item['target'] + suffix)
            if os.path.isfile(path):
                files[field] = path
        out.append((meta, files))
    return out
