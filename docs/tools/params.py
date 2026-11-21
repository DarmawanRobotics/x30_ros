"""Generate the parameter reference of the Tuning Guide from the X30 sources.

python3 docs/tools/params.py     (make -C docs reference)

Every typed parameter comes from its configuration dataclass (x30_utils.config.entries: name,
default, unit, range, choices, live, description), the driver's from the comments of
x30_driver/config/driver_params.yaml. Output: docs/latex/generated/params_<group>.tex, one
longtable per node, read by the Tuning Guide and, through pandoc, by the website. Needs only
Python 3.8 and numpy (some controllers import it); no ROS.
"""

from __future__ import annotations

import importlib
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / 'docs' / 'latex' / 'generated'

for src in sorted(REPO.glob('x30_*/src')):
    sys.path.insert(0, str(src))

from x30_utils.config import entries  # noqa: E402

CONTROLLERS = [
    ('holonomic', 'holonomic', 'HolonomicConfig'),
    ('pure_pursuit', 'pure_pursuit', 'PurePursuitConfig'),
    ('pid', 'pid', 'PidConfig'),
    ('lqr', 'lqr', 'LqrConfig'),
    ('stanley', 'stanley', 'StanleyConfig'),
    ('mppi', 'mppi', 'MppiConfig'),
    ('neupan', 'neupan_mpc', 'NeupanConfig'),
]
POLICIES = [
    ('braitenberg', 'braitenberg', 'BraitenbergConfig'),
    ('follow_gap', 'follow_gap', 'FollowGapConfig'),
    ('potential_field', 'potential_field', 'PotentialFieldConfig'),
    ('vfh', 'vfh', 'VfhConfig'),
]

# group -> [(node, module, class, prefix)]
TABLES = {
    'robot': [('every node (robot/...)', 'x30_utils.robot.profile', 'RobotProfile', 'robot.')],
    'control': [
        ('control_mux_node', 'x30_control.core.config', 'MuxConfig', ''),
        ('control_mux_node: protective field', 'x30_control.core.config', 'FieldConfig', 'field.'),
    ],
    'perception': [
        ('lidar_sector_node', 'x30_perception.core.config', 'PerceptionConfig', ''),
        ('aruco_detector_node', 'x30_perception.core.aruco', 'ArucoConfig', ''),
    ],
    'odometry': [
        ('wait_still (FAST-LIO start gate)', 'x30_odometry.core.config', 'StillConfig', ''),
        ('odometry_monitor_node', 'x30_odometry.core.config', 'MonitorConfig', ''),
    ],
    'localization': [
        ('localization_node', 'x30_localization.core.config', 'LocalizationConfig', ''),
    ],
    'navigation': [
        ('site_manager_node', 'x30_navigation.core.config.sites', 'SiteConfig', ''),
        ('route_recorder_node', 'x30_navigation.core.config.sites', 'RecorderConfig', ''),
        ('path_loader_node', 'x30_navigation.core.config.loader', 'LoaderConfig', ''),
        ('path_follower_node', 'x30_navigation.core.config.follower', 'FollowerConfig', ''),
        ('obstacle_avoidance_node', 'x30_navigation.core.config.avoidance', 'AvoidanceConfig', ''),
    ],
    'controllers': [
        (f'path_follower_node: {name}', f'x30_navigation.core.controllers.{module}', cls,
         f'{name}.')  # fmt: skip
        for name, module, cls in CONTROLLERS
    ],
    'avoidance': [
        (f'obstacle_avoidance_node: {name}', f'x30_navigation.core.avoidance.{module}', cls,
         f'{name}.')  # fmt: skip
        for name, module, cls in POLICIES
    ],
    'docking': [('docking_node', 'x30_navigation.core.config.docking', 'DockingConfig', '')],
    'health': [('health_monitor_node', 'x30_health.core.config', 'HealthConfig', '')],
    'inspection': [('inspection_node', 'x30_inspection.core.config', 'InspectionConfig', '')],
    'recorder': [('recorder_node', 'x30_recorder.core.config', 'RecorderConfig', '')],
    'bridge': [('teleop_udp_node', 'x30_bridge.core.config', 'TeleopConfig', '')],
    'fleet': [('fleet_uplink_node', 'x30_fleet.core.config', 'FleetConfig', '')],
    'sim': [('sim_robot_node', 'x30_sim.core.config', 'SimConfig', '')],
}

SPECIAL = {
    '\\': r'\textbackslash{}',
    '&': r'\&',
    '%': r'\%',
    '$': r'\$',
    '#': r'\#',
    '_': r'\_',
    '{': r'\{',
    '}': r'\}',
    '~': r'\textasciitilde{}',
    '^': r'\textasciicircum{}',
}


def tex(text) -> str:
    """Escape plain text for LaTeX."""
    return ''.join(SPECIAL.get(c, c) for c in str(text))


def unit(text: str) -> str:
    """Typeset a unit such as m/s^2, deg/sqrt(m) or degC."""
    out = re.sub(r'\\textasciicircum\{\}(\d)', r'\\textsuperscript{\1}', tex(text))
    out = out.replace('degC', '\\textdegree{}C').replace('deg', '\\textdegree{}')
    return re.sub(r'sqrt\(([a-z]+)\)', r'$\\sqrt{\\mathrm{\1}}$', out)


def words(text: str) -> str:
    """Escape a description; a long word (a URL, a topic) may break after its slashes."""
    return ' '.join(
        w.replace('/', '/\\allowbreak{}') if len(w) > 18 else w for w in tex(text).split(' ')
    )


def value(v) -> str:
    """Typeset a default value."""
    if isinstance(v, bool):
        return r'\code{true}' if v else r'\code{false}'
    if isinstance(v, float):
        return f'{v:g}'
    if isinstance(v, (list, tuple)):
        if not v:
            return r'\code{[]}'
        # one item at a time, so a long list breaks between its items
        return ', '.join(item(x) for x in v)
    if isinstance(v, str):
        return r'\code{' + v + '}' if v and ' ' not in v and '%' not in v else (
            r'\texttt{"' + tex(v) + '"}')  # fmt: skip
    return tex(v)


def item(x) -> str:
    """Typeset one item of a list default."""
    if isinstance(x, float):
        return f'{x:g}'
    text = str(x)
    return r'\code{' + text + '}' if text and ' ' not in text and '%' not in text else (
        r'\texttt{' + tex(text) + '}')  # fmt: skip


def span(spec) -> str:
    """Typeset the range or the choices of a parameter."""
    if spec.choices:
        return ', '.join(tex(c) for c in spec.choices)
    lo = '' if spec.min is None else f'{spec.min:g}'
    hi = '' if spec.max is None or spec.max >= 1e6 else f'{spec.max:g}'
    return f'{lo}\\,..\\,{hi}' if lo or hi else ''


HEAD = (
    '\\begin{small}\n'
    '\\begin{longtable}{@{}>{\\raggedright}p{0.26\\linewidth}>{\\raggedright}p{0.15\\linewidth}'
    '>{\\raggedright}p{0.07\\linewidth}>{\\raggedright}p{0.1\\linewidth}'
    '>{\\raggedright\\arraybackslash}p{0.3\\linewidth}@{}}\n'
    '\\toprule\n'
    '\\textbf{Parameter} & \\textbf{Default} & \\textbf{Unit} & \\textbf{Range} & '
    '\\textbf{Description} \\\\\n'
    '\\midrule\n\\endhead\n'
    '\\bottomrule\n\\endfoot\n'
)
TAIL = '\\end{longtable}\n\\end{small}\n'


def param_table(rows: list[str]) -> str:
    """Return one longtable of parameter rows."""
    return HEAD + ''.join(rows) + TAIL


def typed_rows(module: str, cls: str, prefix: str) -> list[str]:
    """Return the rows of one configuration dataclass."""
    config = getattr(importlib.import_module(module), cls)
    rows = []
    for entry in entries(config, prefix):
        spec = entry.spec
        mark = '' if spec.live else '$^\\dagger$'
        rows.append(
            f'\\code{{{entry.name}}}{mark} & {value(entry.default)} & {unit(spec.unit)} & '
            f'{span(spec)} & {words(spec.doc)} \\\\\n'
        )
    return rows


KEY = re.compile(r'^([a-z_][a-z0-9_]*):\s*([^#]*?)\s*(?:#\s*(.*))?$')


def driver_rows() -> list[str]:
    """Return the rows of x30_driver/config/driver_params.yaml (top-level keys, inline notes)."""
    rows, note = [], []
    for line in (REPO / 'x30_driver' / 'config' / 'driver_params.yaml').read_text().splitlines():
        if line.startswith('#'):
            text = line.lstrip('#').strip()
            note = [] if text.startswith('---') else [*note, text]
            continue
        match = KEY.match(line)
        if not match:
            note = [] if not line.strip() else note
            continue
        name, default, comment = match.groups()
        note_text = comment or (note[-1] if note else '')
        rows.append(f'\\code{{{name}}} & \\code{{{default or "{}"}}} & & & {words(note_text)} \\\\\n')
        note = []
    return rows


def write(name: str, body: str):
    """Write one generated file, said to be generated."""
    OUT.mkdir(parents=True, exist_ok=True)
    head = '% Generated by docs/tools/params.py from the X30 sources: do not edit.\n'
    (OUT / f'params_{name}.tex').write_text(head + body)


def main():
    """Write every table."""
    for group, tables in TABLES.items():
        parts = []
        for node, module, cls, prefix in tables:
            parts.append(f'\\subsubsection*{{\\code{{{node.split(":")[0]}}}'
                         f'{tex(node[len(node.split(":")[0]):])}}}\n')  # fmt: skip
            parts.append(param_table(typed_rows(module, cls, prefix)))
        write(group, ''.join(parts))
    write('driver', '\\subsubsection*{\\code{x30_driver}}\n' + param_table(driver_rows()))
    print(f'wrote {len(TABLES) + 1} files to {OUT.relative_to(REPO)}')


if __name__ == '__main__':
    main()
