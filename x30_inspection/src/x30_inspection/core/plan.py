"""The inspection plan of a site: <sites_dir>/<site>/inspection.yaml."""

# Inspection points are where the robot stands (map frame), each with its targets: where the PTZ
# camera looks (pan, tilt, zoom in the camera's own convention) and what is checked there.
# Points are matched by position, not by route: every route that stops near a point inspects it.
#
#     points:
#       - name: bay1_disconnector
#         x: 12.40
#         y: -3.10
#         targets:
#           - name: bay1_ds_clamps
#             pan_deg: 35.0
#             tilt_deg: -10.0
#             zoom: 4.0
#             thermal: {enabled: true, strips: 3, max_temp_c: 0.0}
#             acoustic: {enabled: true, duration_sec: 5.0}

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
import os

import yaml

from x30_utils.io import valid_name, write_text_atomic

PLAN_FILE = 'inspection.yaml'
# NETRA's plan of the running patrol (x30_fleet writes it), inspected beside the site's own
NETRA_PLAN_FILE = os.path.join('netra', 'inspection.yaml')
PAN = (-360.0, 360.0)  # deg; the camera wraps the azimuth itself
TILT = (-90.0, 90.0)  # deg, the camera's elevation sign
ZOOM = (1.0, 60.0)


class PlanError(ValueError):
    """The plan file or a requested change is invalid."""


@dataclass(frozen=True)
class Region:
    """A box in fractions of the thermal frame: (x1, y1) - (x2, y2), 0..1 from the top left."""

    name: str
    box: tuple[float, float, float, float]


@dataclass(frozen=True)
class ThermalTarget:
    """What the thermal camera checks at a target."""

    enabled: bool = True
    strips: int = 0  # > 1: N side-by-side strips compared with each other (three phases)
    regions: tuple[Region, ...] = ()  # explicit boxes, compared with each other
    max_temp_c: float = 0.0  # > 0: own critical absolute limit


@dataclass(frozen=True)
class AcousticTarget:
    """What the acoustic camera checks at a target."""

    enabled: bool = False
    duration_sec: float = 0.0  # 0: the configured default


@dataclass(frozen=True)
class Target:
    """One view of the PTZ camera at an inspection point."""

    name: str
    pan_deg: float
    tilt_deg: float
    zoom: float = 1.0
    thermal: ThermalTarget = field(default_factory=ThermalTarget)
    acoustic: AcousticTarget = field(default_factory=AcousticTarget)


@dataclass(frozen=True)
class Point:
    """Where the robot stands for its targets (map frame)."""

    name: str
    x: float
    y: float
    targets: tuple[Target, ...] = ()


@dataclass(frozen=True)
class Plan:
    """Every inspection point of a site."""

    points: tuple[Point, ...] = ()

    @property
    def targets(self) -> int:
        """Return the number of targets over every point."""
        return sum(len(p.targets) for p in self.points)

    def nearest(self, x: float, y: float, radius: float) -> Point | None:
        """Return the point closest to (x, y) within radius, or None."""
        best, best_d = None, radius
        for point in self.points:
            d = math.hypot(point.x - x, point.y - y)
            if d <= best_d:
                best, best_d = point, d
        return best

    def find(self, name: str) -> Point | None:
        """Return the point of a name, or None."""
        return next((p for p in self.points if p.name == name), None)


def plan_path(sites_dir: str, site: str, name: str = PLAN_FILE) -> str:
    """Return a plan file of a site; the site name is checked, never a path."""
    if not valid_name(site):
        raise PlanError(f'site {site!r}: use 1-48 of a-z, 0-9, _ and -')
    return os.path.join(os.path.expanduser(sites_dir), site, name)


def load_site(sites_dir: str, site: str) -> Plan:
    """Return the site's own plan with NETRA's (its points after, their names kept apart)."""
    own = load(plan_path(sites_dir, site))
    netra = load(plan_path(sites_dir, site, NETRA_PLAN_FILE))
    names = {p.name for p in own.points}
    return Plan(own.points + tuple(p for p in netra.points if p.name not in names))


def _number(data: dict, key: str, default: float, low: float, high: float) -> float:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PlanError(f'{key}: {value!r} is not a number')
    if not low <= value <= high:
        raise PlanError(f'{key}: {value} outside [{low}, {high}]')
    return float(value)


def _name(data: dict, what: str) -> str:
    name = str(data.get('name', ''))
    if not valid_name(name):
        raise PlanError(f'{what} name {name!r}: use 1-48 of a-z, 0-9, _ and -')
    return name


def _region(data: dict) -> Region:
    box = data.get('box')
    name = data.get('name')
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise PlanError(f'region {name!r}: box needs [x1, y1, x2, y2] (fractions of the frame)')
    x1, y1, x2, y2 = (_number({'box': v}, 'box', 0.0, 0.0, 1.0) for v in box)
    if x2 <= x1 or y2 <= y1:
        raise PlanError(f'region {name!r}: box needs x1 < x2 and y1 < y2')
    return Region(_name(data, 'region'), (x1, y1, x2, y2))


def _target(data: dict) -> Target:
    thermal = data.get('thermal') or {}
    acoustic = data.get('acoustic') or {}
    if not isinstance(thermal, dict) or not isinstance(acoustic, dict):
        raise PlanError('thermal and acoustic must be mappings')
    regions = tuple(_region(r) for r in thermal.get('regions') or ())
    if len({r.name for r in regions}) != len(regions):
        raise PlanError(f'target {data.get("name")!r}: region names must be unique')
    return Target(
        name=_name(data, 'target'),
        pan_deg=_number(data, 'pan_deg', 0.0, *PAN),
        tilt_deg=_number(data, 'tilt_deg', 0.0, *TILT),
        zoom=_number(data, 'zoom', 1.0, *ZOOM),
        thermal=ThermalTarget(
            enabled=bool(thermal.get('enabled', True)),
            strips=int(_number(thermal, 'strips', 0, 0, 16)),
            regions=regions,
            max_temp_c=_number(thermal, 'max_temp_c', 0.0, 0.0, 500.0),
        ),
        acoustic=AcousticTarget(
            enabled=bool(acoustic.get('enabled', False)),
            duration_sec=_number(acoustic, 'duration_sec', 0.0, 0.0, 60.0),
        ),
    )


def _point(data: dict) -> Point:
    if not isinstance(data, dict):
        raise PlanError(f'a point is a mapping, got {data!r}')
    targets = tuple(_target(t) for t in data.get('targets') or ())
    names = [t.name for t in targets]
    if len(set(names)) != len(names):
        raise PlanError(f'point {data.get("name")!r}: target names must be unique')
    x = _number(data, 'x', 0.0, -1e5, 1e5)
    return Point(_name(data, 'point'), x, _number(data, 'y', 0.0, -1e5, 1e5), targets)


def parse(data) -> Plan:
    """Return the plan of loaded YAML (None: empty)."""
    if data is None:
        return Plan()
    if not isinstance(data, dict) or not isinstance(data.get('points') or [], list):
        raise PlanError('expected {points: [...]}')
    points = tuple(_point(p) for p in data.get('points') or [])
    names = [p.name for p in points]
    if len(set(names)) != len(names):
        raise PlanError('point names must be unique')
    return Plan(points)


def load(path: str) -> Plan:
    """Read a plan file; a missing file is an empty plan."""
    if not os.path.isfile(path):
        return Plan()
    try:
        with open(path, encoding='utf-8') as f:
            return parse(yaml.safe_load(f))
    except (OSError, yaml.YAMLError) as error:
        raise PlanError(f'{path}: {error}') from error


def dump(plan: Plan) -> str:
    """Return the YAML text of a plan."""
    points = [asdict(p) for p in plan.points]
    for point in points:
        for target in point['targets']:
            target['thermal']['regions'] = [
                {'name': r['name'], 'box': list(r['box'])} for r in target['thermal']['regions']
            ]
    return yaml.safe_dump({'points': points}, sort_keys=False, default_flow_style=None)


def save(path: str, plan: Plan):
    """Write a plan atomically (the previous one kept as .bak)."""
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            write_text_atomic(path + '.bak', f.read())
    write_text_atomic(path, dump(plan))


def add_target(plan: Plan, point_name: str, x: float, y: float, radius: float, target: Target):
    """Return (plan, point) with target added to the named point, the near one or a new one."""
    point = plan.find(point_name) if point_name else plan.nearest(x, y, radius)
    if point is None:
        name = point_name or target.name
        if not valid_name(name):
            raise PlanError(f'point name {name!r}: use 1-48 of a-z, 0-9, _ and -')
        if plan.find(name) is not None:
            raise PlanError(f'point {name!r} exists elsewhere: name the point')
        point = Point(name, round(x, 3), round(y, 3))
        points = (*plan.points, point)
    else:
        points = plan.points
    if any(t.name == target.name for t in point.targets):
        raise PlanError(f'point {point.name!r} has a target {target.name!r} already')
    grown = replace(point, targets=(*point.targets, target))
    return Plan(tuple(grown if p.name == point.name else p for p in points)), grown
