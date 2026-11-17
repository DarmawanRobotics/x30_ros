"""In-process tests of control_mux_node: forwarding, e-stop, re-arm, field, fail-safe."""

from __future__ import annotations

import time

from conftest import N, twist
import pytest
import rospy
from std_msgs.msg import Bool, String

from x30_msgs.msg import ControlStatus, HealthStatus
from x30_utils.config import ConfigError
from x30_utils.testing.harness import connected, wait_for

MOVE = twist(0.3)


def moving(msgs) -> list:
    return [m for m in msgs if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6]


def drive(rig, pub, msg=MOVE, seconds: float = 0.3) -> list:
    start = len(rig.out.messages)
    rig.stream(pub, msg, seconds)
    time.sleep(0.05)
    return rig.outputs_since(start)


def rearm(rig, pub, timeout: float = 3.0):
    """Send neutral commands until the active source may drive again (rearm_neutral_count)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rig.stream(pub, twist(), 0.1)
        if rig.last_status().armed:
            return
    raise AssertionError('neutral commands did not re-arm the source')


def test_auto_forwards_navigation_and_status_reports(make_rig):
    rig = make_rig()
    out = drive(rig, rig.nav)
    assert out and out[-1].linear.x == pytest.approx(0.3)
    status = rig.last_status()
    assert status.mode == ControlStatus.MODE_AUTO and not status.estop
    assert status.header.frame_id == 'base_link'
    assert not moving(drive(rig, rig.remote['handheld']))  # REMOTE sources are ignored


def test_estop_service_holds_zero_and_release_needs_rearm(make_rig):
    rig = make_rig()
    reply = rig.engage()
    assert reply.success and rig.wait_status(lambda s: s.estop and not s.armed)
    assert list(rig.last_status().estop_reasons) == ['service']
    out = drive(rig, rig.nav, seconds=0.5)
    assert len(out) >= 4 and not moving(out)  # hold stream at estop_hold_hz, nothing else
    assert rig.release().success
    assert rig.wait_status(lambda s: not s.estop and not s.armed)
    assert not moving(drive(rig, rig.nav, seconds=0.6))  # navigation still pushing: no motion
    time.sleep(0.7)  # navigation silent for nav_timeout_sec: re-armed
    assert rig.wait_status(lambda s: s.armed)
    assert moving(drive(rig, rig.nav))


def test_estop_request_topic_appends_reasons(make_rig):
    rig = make_rig()
    rig.request.publish(String(data='bumper'))
    assert rig.wait_status(lambda s: list(s.estop_reasons) == ['bumper'])
    rig.request.publish(String(data='fleet'))
    rig.request.publish(String(data='bumper'))
    assert rig.wait_status(lambda s: list(s.estop_reasons) == ['bumper', 'fleet'])


def test_hardware_estop_blocks_release(make_rig):
    rig = make_rig()
    rig.hw.publish(Bool(data=True))
    assert rig.wait_status(lambda s: s.estop and 'hardware e-stop' in s.estop_reasons)
    reply = rig.release()
    assert not reply.success and rig.last_status().estop
    rig.hw.publish(Bool(data=False))
    time.sleep(0.2)
    assert rig.last_status().estop  # the button alone never releases
    assert rig.release().success
    assert rig.wait_status(lambda s: not s.estop)


def test_remote_sources_and_set_mode(make_rig):
    rig = make_rig()
    assert rig.mode('remote', 'dashboard').success
    assert rig.last_status().remote_source == 'dashboard'
    assert not rig.mode('remote', 'joystick').success
    assert not rig.mode('manual').success
    assert not moving(drive(rig, rig.remote['dashboard']))  # held while switching: re-arm first
    rearm(rig, rig.remote['dashboard'])
    assert not moving(drive(rig, rig.remote['handheld']))
    assert not moving(drive(rig, rig.nav))
    start = len(rig.out.messages)
    for _ in range(5):
        rig.remote['dashboard'].publish(MOVE)
        assert rig.mode('remote', 'dashboard').success  # no-op, no zero in between
        time.sleep(0.03)
    time.sleep(0.05)
    out = rig.outputs_since(start)
    assert len(out) >= 5 and all(m.linear.x > 0.0 for m in out[:5])


def test_watchdog_zeroes_a_silent_source(make_rig):
    rig = make_rig()
    drive(rig, rig.nav)
    time.sleep(0.8)
    out = rig.out.messages
    last_move = max(i for i, m in enumerate(out) if m.linear.x > 0.0)
    after = out[last_move + 1 :]
    assert len(after) == 1 and not moving(after)  # exactly one zero once navigation is silent


def test_field_stops_navigation_toward_an_obstacle_and_reports_it(make_rig):
    rig = make_rig()
    rig.ranges[N // 2] = 0.5  # straight ahead
    time.sleep(0.3)
    out = drive(rig, rig.nav)
    assert out and out[-1].linear.x == 0.0
    assert rig.last_status().field == ControlStatus.FIELD_STOP
    out = drive(rig, rig.nav, twist(-0.3))
    assert out[-1].linear.x == pytest.approx(-0.3)


def test_field_limits_rotation_beside_an_obstacle(make_rig):
    rig = make_rig()
    rig.ranges[N // 2 + 9] = 0.47  # 45 deg left, inside the swept ring
    time.sleep(0.3)
    out = drive(rig, rig.nav, twist(wz=1.0))
    assert out and abs(out[-1].angular.z) < 1.0


def test_field_on_in_remote_by_default(make_rig):
    rig = make_rig()
    rig.ranges[N // 2] = 0.5
    rig.mode('remote', 'handheld')
    rearm(rig, rig.remote['handheld'])
    out = drive(rig, rig.remote['handheld'])
    assert out and out[-1].linear.x == 0.0


def test_blind_scan_stops_auto_and_creeps_remote(make_rig):
    rig = make_rig()
    rig.ranges = [0.0] * N
    assert rig.wait_status(lambda s: s.field == ControlStatus.FIELD_BLIND)
    assert not moving(drive(rig, rig.nav))
    rig.mode('remote', 'handheld')
    rearm(rig, rig.remote['handheld'])
    out = drive(rig, rig.remote['handheld'], twist(0.5))
    assert out and out[-1].linear.x == pytest.approx(0.15, abs=1e-3)  # blind sensor: creep
    rig.ranges = [4.0] * N
    rig.ranges[N // 2] = 0.0  # one blocked sector straight ahead
    time.sleep(0.3)
    out = drive(rig, rig.remote['handheld'], twist(0.5))
    assert out and out[-1].linear.x == 0.0  # an obstacle at the robot ahead
    out = drive(rig, rig.remote['handheld'], twist(-0.5))
    assert out and out[-1].linear.x == pytest.approx(-0.15, abs=1e-3)


def test_invalid_or_stale_scan_is_stale(make_rig):
    rig = make_rig()
    rig.frame = 'livox_frame'
    assert rig.wait_status(lambda s: s.field == ControlStatus.FIELD_STALE)
    assert not moving(drive(rig, rig.nav))
    rig.frame = 'base_link'
    assert rig.wait_status(lambda s: s.field == ControlStatus.FIELD_CLEAR)
    rig.scanning = False
    assert rig.wait_status(lambda s: s.field == ControlStatus.FIELD_STALE)
    rig.mode('remote', 'handheld')
    rearm(rig, rig.remote['handheld'])
    out = drive(rig, rig.remote['handheld'], twist(0.5))
    assert out and out[-1].linear.x == pytest.approx(0.15, abs=1e-3)


def test_non_finite_and_envelope(make_rig):
    rig = make_rig(**{'robot.limits.max_vx': 1.0, 'robot.limits.max_vy': 0.5})
    out = drive(rig, rig.nav, twist(float('nan')))
    assert out and not moving(out)
    msg = twist(0.3)
    msg.angular.y = float('inf')
    assert not moving(drive(rig, rig.nav, msg))
    out = drive(rig, rig.nav, twist(2.0, 1.0))
    assert out[-1].linear.x == pytest.approx(1.0) and out[-1].linear.y == pytest.approx(0.5)


def test_health_limit_scales_commands(make_rig):
    rig = make_rig(use_health_limit=True)
    assert connected(rig.health)
    rig.health.publish(HealthStatus(speed_scale=0.5))
    time.sleep(0.2)
    out = drive(rig, rig.nav, twist(0.4))
    assert out[-1].linear.x == pytest.approx(0.2)
    assert rig.last_status().speed_scale == pytest.approx(0.5)


def test_safety_parameters_are_read_only(make_rig):
    rig = make_rig()
    rospy.set_param(f'{rig.ns}/nav_timeout_sec', 2.0)
    rospy.set_param(f'{rig.ns}/field/margin', 0.0)
    rospy.set_param(f'{rig.ns}/robot/limits/max_vx', 3.0)
    # the node's own poll or this one: each change is noted once
    rig.mux._binder.poll()
    assert wait_for(lambda: len(rig.mux.param_notes) == 3, 3.0), rig.mux.param_notes
    notes = rig.mux.param_notes
    assert all('read-only' in n for n in notes), notes
    assert rig.mux._mux.nav_timeout_sec == 0.5 and rig.mux._field_cfg.margin == 0.15
    assert rig.mux._robot.limits.max_vx == 0.8


@pytest.mark.parametrize(
    'params',
    [
        {'robot.limits.max_decel': 0.0},
        {'nav_timeout_sec': 0.0},
        {'field.margin': -0.1},
        {'robot.limits.max_wz': float('nan')},
        {'remote_sources': ['bad name']},
    ],
)
def test_bad_values_refuse_to_start(make_rig, params):
    with pytest.raises(ConfigError):
        make_rig(ready=False, **params)


def test_callback_error_engages_the_estop(make_rig):
    rig = make_rig()

    def broken(*_):
        raise RuntimeError('boom')

    rig.mux._gate.apply = broken
    out = drive(rig, rig.nav)
    assert out and not moving(out)
    assert rig.wait_status(lambda s: s.estop and 'internal error' in s.estop_reasons)


def test_shutdown_hook_publishes_zeros(make_rig):
    rig = make_rig()
    drive(rig, rig.nav)
    start = len(rig.out.messages)
    rig.mux.run_shutdown_hooks()
    assert wait_for(lambda: len(rig.out.messages) - start >= 3, 2.0)
    assert not moving(rig.outputs_since(start))
