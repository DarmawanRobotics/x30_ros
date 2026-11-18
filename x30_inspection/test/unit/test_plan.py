"""The inspection plan file: parsing, validation, matching by position, teaching targets."""

from __future__ import annotations

import pytest
import yaml

from x30_inspection.core.plan import (
    Plan,
    PlanError,
    Target,
    ThermalTarget,
    add_target,
    dump,
    load,
    parse,
    plan_path,
    save,
)

TEXT = """
points:
  - name: bay1
    x: 12.4
    y: -3.1
    targets:
      - name: ds_clamps
        pan_deg: 35.0
        tilt_deg: -10.0
        zoom: 4.0
        thermal: {strips: 3}
        acoustic: {enabled: true, duration_sec: 5}
      - name: ct_terminal
        pan_deg: -20.0
        tilt_deg: 5.0
        thermal:
          regions:
            - {name: r, box: [0.05, 0.1, 0.3, 0.6]}
            - {name: s, box: [0.35, 0.1, 0.6, 0.6]}
  - name: bay2
    x: 20.0
    y: -3.0
"""


def test_parse_the_documented_layout():
    plan = parse(yaml.safe_load(TEXT))
    assert [p.name for p in plan.points] == ['bay1', 'bay2']
    assert plan.targets == 2
    ds = plan.points[0].targets[0]
    assert (ds.pan_deg, ds.tilt_deg, ds.zoom) == (35.0, -10.0, 4.0)
    assert ds.thermal.enabled and ds.thermal.strips == 3
    assert ds.acoustic.enabled and ds.acoustic.duration_sec == 5.0
    ct = plan.points[0].targets[1]
    assert ct.zoom == 1.0 and not ct.acoustic.enabled
    assert [r.box for r in ct.thermal.regions] == [(0.05, 0.1, 0.3, 0.6), (0.35, 0.1, 0.6, 0.6)]


def test_round_trip(tmp_path):
    plan = parse(yaml.safe_load(TEXT))
    path = tmp_path / 'inspection.yaml'
    save(str(path), plan)
    assert load(str(path)) == plan
    save(str(path), Plan())
    assert (tmp_path / 'inspection.yaml.bak').read_text() == dump(plan)


@pytest.mark.parametrize(
    ('text', 'error'),
    [
        ('points: [{name: "Bay 1", x: 0, y: 0}]', 'point name'),
        ('points: [{name: a, x: 0, y: 0}, {name: a, x: 1, y: 1}]', 'unique'),
        ('points: [{name: a, x: .nan, y: 0}]', 'not a number'),
        ('points: [{name: a, x: 0, y: 0, targets: [{name: t, tilt_deg: 95}]}]', 'tilt_deg'),
        ('points: [{name: a, x: 0, y: 0, targets: [{name: t, zoom: 0.5}]}]', 'zoom'),
        ('points: [{name: a, x: 0, y: 0, targets: [{name: t}, {name: t}]}]', 'unique'),
        (
            'points: [{name: a, x: 0, y: 0, targets: [{name: t, thermal: '
            '{regions: [{name: r, box: [0.5, 0.5, 0.1, 0.1]}]}}]}]',
            'x1 < x2',
        ),
        (
            'points: [{name: a, x: 0, y: 0, targets: [{name: t, thermal: '
            '{regions: [{name: r, box: [10, 10, 100, 120]}]}}]}]',
            'box: 10',
        ),
        (
            'points: [{name: a, x: 0, y: 0, targets: [{name: t, thermal: {regions: ['
            '{name: r, box: [0, 0, 0.5, 1]}, {name: r, box: [0.5, 0, 1, 1]}]}}]}]',
            'region names',
        ),
        ('points: [7]', 'mapping'),
        ('[1, 2]', 'points'),
    ],
)
def test_invalid_files_are_refused(text, error):
    with pytest.raises(PlanError, match=error):
        parse(yaml.safe_load(text))


def test_missing_file_is_an_empty_plan(tmp_path):
    assert load(str(tmp_path / 'none.yaml')) == Plan()
    broken = tmp_path / 'broken.yaml'
    broken.write_text('points: [')
    with pytest.raises(PlanError):
        load(str(broken))


def test_site_names_never_become_paths():
    assert plan_path('/sites', 'gi_cawang') == '/sites/gi_cawang/inspection.yaml'
    with pytest.raises(PlanError, match='site'):
        plan_path('/sites', '../etc')


def test_nearest_point_within_the_radius():
    plan = parse(yaml.safe_load(TEXT))
    assert plan.nearest(12.0, -3.0, 1.5).name == 'bay1'
    assert plan.nearest(19.0, -3.0, 1.5).name == 'bay2'
    assert plan.nearest(16.0, -3.0, 1.5) is None


def test_teach_targets():
    plan = parse(yaml.safe_load(TEXT))
    new = Target('busbar_joint', 0.0, -15.0, 2.0, ThermalTarget(strips=3))
    grown, point = add_target(plan, '', 12.2, -3.2, 1.5, new)
    assert point.name == 'bay1' and len(point.targets) == 3
    assert grown.points[1] == plan.points[1]
    fresh, point = add_target(grown, '', 40.0, 2.0, 1.5, Target('trafo_bushing', 0, 0))
    assert (point.name, point.x, point.y) == ('trafo_bushing', 40.0, 2.0)
    assert len(fresh.points) == 3
    with pytest.raises(PlanError, match='already'):
        add_target(fresh, '', 12.2, -3.2, 1.5, new)
    named, point = add_target(plan, 'bay2', 0.0, 0.0, 1.5, Target('cable_end', 0, 0))
    assert point.name == 'bay2' and point.x == 20.0
    assert named.targets == 3


def test_new_point_named_after_an_existing_one_is_refused():
    plan = parse(yaml.safe_load(TEXT))
    with pytest.raises(PlanError, match='exists'):
        add_target(plan, '', 99.0, 0.0, 1.5, Target('bay1', 0, 0))
