"""Typed states (never strings), expected nodes with namespaces (#13) and the safety scan."""

from __future__ import annotations

import math

import numpy as np

from x30_health.core.checks.base import Level
from x30_health.core.checks.nodes import NodesCheck, qualified
from x30_health.core.checks.scan import SafetyScanCheck, blocked_count
from x30_health.core.checks.state import (
    ControlInput,
    Field,
    LocalizationInput,
    LocalizationState,
    OdometryInput,
    OdometryState,
    StateCheck,
    control_issue,
    localization_issue,
    odometry_issue,
)
from x30_health.core.config import NodesConfig, StateConfig


def test_odometry_states():
    assert odometry_issue(OdometryInput(OdometryState.OK)).level is Level.OK
    lost = odometry_issue(OdometryInput(OdometryState.LOST, 'no odometry for 2.0 s'))
    assert lost.level is Level.ERROR and lost.message == 'odometry lost: no odometry for 2.0 s'
    assert odometry_issue(OdometryInput(OdometryState.DEGRADED)).level is Level.WARN
    assert odometry_issue(OdometryInput(OdometryState.INITIALIZING)).level is Level.WARN
    assert odometry_issue(OdometryInput(42)).level is Level.WARN


def test_not_localized_is_a_warning():
    """#9: health said ok while the robot was not localized (sigma inf while NONE)."""
    none = localization_issue(LocalizationInput(LocalizationState.NONE, math.inf, math.inf))
    assert none.level is Level.WARN and none.message == 'not localized'
    uncertain = localization_issue(LocalizationInput(LocalizationState.UNCERTAIN, 0.6, 0.1, 'x'))
    assert uncertain.level is Level.WARN and uncertain.message == 'localization uncertain: x'
    ok = localization_issue(LocalizationInput(LocalizationState.LOCALIZED, 0.05, 0.01))
    assert ok.level is Level.OK and ok.values['sigma_xy'] == 0.05
    assert localization_issue(LocalizationInput(9)).level is Level.WARN


def test_control_states():
    estop = control_issue(ControlInput(True, ('hw button', 'remote')))
    assert estop.level is Level.ERROR and 'hw button, remote' in estop.message
    assert control_issue(ControlInput(False, (), Field.STALE)).level is Level.ERROR
    assert control_issue(ControlInput(False, (), Field.BLIND)).level is Level.ERROR
    for field in (Field.OFF, Field.CLEAR, Field.LIMITED, Field.STOP):
        assert control_issue(ControlInput(False, (), field)).level is Level.OK


def test_missing_stale_and_disabled_statuses():
    state = StateCheck(StateConfig(control_topic='', timeout_sec=3.0))
    issues = state.run(0.0)
    assert [i.name for i in issues] == ['state:odometry', 'state:localization']
    assert all(i.level is Level.WARN and 'received' in i.message for i in issues)
    state.on_odometry(10.0, OdometryInput(OdometryState.OK))
    state.on_localization(10.0, LocalizationInput(LocalizationState.LOCALIZED))
    assert all(i.ok for i in state.run(12.0))
    stale = state.run(14.0)[0]
    assert stale.level is Level.WARN and 'stale' in stale.message


def test_nodes_with_namespaces():
    """#13: '/robot_a/control_mux_node' is not '/robot_b/control_mux_node'."""
    expected = ('/robot_a/control_mux_node', '/health_monitor_node', '/robot_a/path_node')
    nodes = NodesCheck(NodesConfig(expected=expected), grace_until=30.0)
    nodes.update(
        [('control_mux_node', '/robot_b'), ('health_monitor_node', '/'), ('path_node', 'robot_a')]
    )
    during = {i.name: i for i in nodes.run(10.0)}
    assert during['node:/robot_a/control_mux_node'].level is Level.WARN  # start-up grace
    after = {i.name: i for i in nodes.run(31.0)}
    assert after['node:/robot_a/control_mux_node'].level is Level.ERROR
    assert after['node:/health_monitor_node'].ok
    assert after['node:/robot_a/path_node'].ok


def test_node_running_twice():
    nodes = NodesCheck(NodesConfig(expected=('/a/n',)), grace_until=0.0)
    nodes.update([('n', '/a'), ('n', '/a/')])
    (issue,) = nodes.run(1.0)
    assert issue.level is Level.WARN and 'runs 2 times' in issue.message


def test_qualified_names():
    assert qualified('n', '/') == '/n'
    assert qualified('n', '') == '/n'
    assert qualified('n', '/a/b') == '/a/b/n'


def test_safety_scan():
    scan = SafetyScanCheck('perception/safety_scan', timeout_sec=1.0, grace_until=5.0)
    assert scan.run(1.0)[0].level is Level.WARN  # not yet, during the grace
    assert scan.run(6.0)[0].level is Level.ERROR
    scan.on_scan(10.0, np.full(36, 5.0))
    assert scan.run(10.5)[0].ok
    stale = scan.run(11.5)[0]
    assert stale.level is Level.ERROR and 'stale' in stale.message
    scan.on_scan(12.0, np.zeros(36))
    assert 'blind' in scan.run(12.0)[0].message
    scan.on_scan(13.0, [])
    assert scan.run(13.0)[0].level is Level.ERROR  # empty scan: blind
    partial = np.full(36, 5.0)
    partial[:3] = [0.0, np.nan, -1.0]
    scan.on_scan(14.0, partial)
    issue = scan.run(14.0)[0]
    assert issue.level is Level.WARN and issue.message == 'safety scan: 3 of 36 sectors blocked'


def test_blocked_count():
    assert blocked_count([np.nan, 0.0, 1.0, np.inf]) == (2, 4)
