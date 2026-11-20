"""Reports on the robot: one folder per inspected point and a daily findings table."""

#     <reports_dir>/<YYYYMMDD>/<HHMMSS>_<point>/report.json, <target>_thermal.jpg, ...
#     <reports_dir>/<YYYYMMDD>/findings.csv

from __future__ import annotations

import csv
from datetime import datetime
import os

from x30_inspection.core.runner import PointResult
from x30_inspection.core.thermal import Severity, worst
from x30_utils.io import write_json_atomic

CSV_FIELDS = (
    'time', 'site', 'point', 'target', 'sensor', 'criterion', 'label', 'severity',
    'value', 'reference', 'delta', 'unit', 'message', 'report',
)  # fmt: skip


def report_dir(root: str, when: datetime, point: str) -> str:
    """Return a new folder for the report of a point (created)."""
    day = os.path.join(os.path.expanduser(root), when.strftime('%Y%m%d'))
    base = os.path.join(day, f'{when.strftime("%H%M%S")}_{point}')
    path, n = base, 1
    while os.path.exists(path):
        n += 1
        path = f'{base}_{n}'
    os.makedirs(path)
    return path


def evidence_prefix(directory: str, target: str) -> str:
    """Return the file prefix of a target's evidence in a report folder."""
    return os.path.join(directory, target)


def to_dict(
    result: PointResult, site: str, x: float, y: float, when: datetime, ambient_c: float = 0.0
) -> dict:
    """Return the JSON of a point's report."""
    return {
        'time': when.isoformat(timespec='seconds'),
        'stamp': round(when.timestamp(), 3),
        'site': site,
        'point': result.point,
        'x': round(x, 3),
        'y': round(y, 3),
        'state': result.state,
        'severity': worst(result.findings).name.lower(),
        'duration_sec': round(result.duration_sec, 1),
        'ambient_c': round(ambient_c, 1),
        'message': result.message,
        'targets': [
            {
                'target': t.target,
                'measured': t.measured,
                'errors': list(t.errors),
                'files': [os.path.basename(f) for f in t.files],
                'readings': [
                    {
                        'region': r.region,
                        'max_c': round(r.temp_max, 2),
                        'min_c': round(r.temp_min, 2),
                    }
                    for r in t.readings
                ],
                'findings': [
                    {
                        'sensor': f.sensor,
                        'criterion': f.criterion,
                        'label': f.label,
                        'severity': f.severity.name.lower(),
                        'value': round(f.value, 2),
                        'reference': round(f.reference, 2),
                        'delta': round(f.delta, 2),
                        'unit': f.unit,
                        'message': f.message,
                    }
                    for f in t.findings
                ],
            }
            for t in result.targets
        ],
    }


def write(directory: str, report: dict):
    """Write report.json into its folder and append its findings to the day's table."""
    write_json_atomic(os.path.join(directory, 'report.json'), report, backup=False)
    table = os.path.join(os.path.dirname(directory), 'findings.csv')
    new = not os.path.isfile(table)
    with open(table, 'a', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            writer.writeheader()
        for target in report['targets']:
            for finding in target['findings']:
                writer.writerow(
                    {
                        'time': report['time'],
                        'site': report['site'],
                        'point': report['point'],
                        'target': target['target'],
                        'report': os.path.basename(directory),
                        **finding,
                    }
                )


def severity_name(value: int) -> str:
    """Return the lower-case name of a severity number."""
    return Severity(value).name.lower()
