"""TF diagnosis (T1-T5): clock offset vs load vs stopped publisher, and the NTP offset."""

from __future__ import annotations

from x30_health.core.checks.base import Level
from x30_health.core.checks.clock import ClockCheck, parse_chrony_tracking
from x30_health.core.checks.tf import TfCheck
from x30_health.core.config import ClockConfig, TfConfig

CHRONY = """Reference ID    : C0A80001 (gateway)
Stratum         : 3
System time     : 0.000012345 seconds slow of NTP time
Last offset     : +0.000001000 seconds
Leap status     : Normal
"""


def run_edges(ages, step=1.0, parent='camera_init', child='body', config=None):
    """Feed ages (s) sampled every step seconds while the edge keeps updating."""
    tf = TfCheck(config or TfConfig())
    for k, age in enumerate(ages):
        tf.observe(100.0 + k * step, {child: (parent, age)})
    return tf, {i.name: i for i in tf.run(0.0)}


def test_fresh_edge_is_ok():
    _, issues = run_edges([0.02, 0.05, 0.03])
    issue = issues['tf:camera_init->body']
    assert issue.ok and issue.values['age_sec'] == 0.03


def test_steady_late_edge_is_a_clock_offset():
    """T1-T5: 'camera_init->body 1.8 s old' with steady ages points at the source clock."""
    _, issues = run_edges([1.8, 1.85, 1.79, 1.82, 1.8])
    issue = issues['tf:camera_init->body']
    assert issue.level is Level.WARN and issue.values['cause'] == 'clock'
    assert issue.message == 'camera_init->body 1.8 s old, steady (source clock offset?)'


def test_growing_ages_are_load():
    _, issues = run_edges([0.1, 0.6, 1.1, 1.5])
    issue = issues['tf:camera_init->body']
    assert issue.values['cause'] == 'load'
    assert 'varying 0.1-1.5 s (system load?)' in issue.message


def test_stopped_publisher():
    tf = TfCheck(TfConfig())
    tf.observe(100.0, {'body': ('camera_init', 0.05)})
    tf.observe(101.0, {'body': ('camera_init', 1.05)})  # same stamp, one second later
    (issue,) = tf.run(0.0)
    assert issue.values['cause'] == 'stopped' and 'not updated' in issue.message
    tf.observe(102.0, {})
    (gone,) = tf.run(0.0)
    assert gone.level is Level.WARN and 'gone' in gone.message


def test_future_stamps_and_few_samples():
    _, issues = run_edges([-0.8])
    assert issues['tf:camera_init->body'].values['cause'] == 'clock'
    assert 'in the future' in issues['tf:camera_init->body'].message
    _, issues = run_edges([0.9, 1.4])
    assert issues['tf:camera_init->body'].values['cause'] == 'late'


def test_window_forgets_old_samples():
    tf, _ = run_edges([0.1] * 3 + [1.8] * 4, config=TfConfig(window_sec=3.0))
    (issue,) = tf.run(0.0)
    assert issue.values['cause'] == 'clock'  # the fresh samples left the window


def test_required_edges_and_chain():
    tf = TfCheck(TfConfig(edges=('map->odom',)), chain=('odom', 'base_link'))
    tf.observe(1.0, {'/body': ('/camera_init', 0.01)}, chain_ok=False)
    issues = {i.name: i for i in tf.run(0.0)}
    assert issues['tf:map->odom'].message == 'map->odom: no transform'
    assert issues['tf:camera_init->body'].ok  # leading slashes stripped
    assert issues['tf:chain odom->base_link'].level is Level.WARN
    tf.observe(2.0, {'odom': ('map', 0.0)}, chain_ok=None)  # chain not checked
    assert 'tf:chain odom->base_link' not in {i.name for i in tf.run(0.0)}


def test_chrony_parsing():
    assert parse_chrony_tracking(CHRONY) == (-0.000012345, True)
    fast = CHRONY.replace('0.000012345 seconds slow', '0.25 seconds fast')
    assert parse_chrony_tracking(fast) == (0.25, True)
    unsynced = CHRONY.replace('Normal', 'Not synchronised')
    assert parse_chrony_tracking(unsynced)[1] is False
    assert parse_chrony_tracking('garbage') == (None, False)


def test_clock_check_runs_the_command_once_per_period():
    calls = []

    def runner(command, timeout):
        calls.append(command)
        return CHRONY.replace('0.000012345 seconds slow', '0.2 seconds fast')

    clock = ClockCheck(ClockConfig(enabled=True, period_sec=10.0), runner)
    issue = clock.run(0.0)[0]
    assert issue.level is Level.WARN and issue.message == 'clock 0.200 s fast of NTP'
    clock.run(5.0)
    assert len(calls) == 1
    clock.run(10.0)
    assert len(calls) == 2


def test_clock_check_failures():
    assert 'failed' in ClockCheck(ClockConfig(), lambda c, t: None).run(0.0)[0].message
    assert 'unreadable' in ClockCheck(ClockConfig(), lambda c, t: 'x').run(0.0)[0].message
    unsynced = CHRONY.replace('Normal', 'Not synchronised')
    issue = ClockCheck(ClockConfig(), lambda c, t: unsynced).run(0.0)[0]
    assert issue.level is Level.WARN and 'not synchronised' in issue.message
    assert ClockCheck(ClockConfig(), lambda c, t: CHRONY).run(0.0)[0].ok


def test_missing_command_fails_cleanly():
    from x30_health.core.checks.clock import run_command

    assert run_command('/nonexistent/chronyc tracking', 1.0) is None
    assert run_command('', 1.0) is None
