from __future__ import annotations

import pytest

from x30_control.core.arbiter import HARDWARE_REASON, HW_SILENT_REASON, NAV, Arbiter, Mode
from x30_control.core.command import ZERO, Command
from x30_control.core.config import MuxConfig
from x30_utils.config import ConfigError, build

CFG = MuxConfig()
MOVE = Command(0.4, 0.0, 0.2)
STILL = Command(0.0, 0.0, 0.0)


def armed_auto(t: float = 0.0) -> Arbiter:
    """An AUTO arbiter whose navigation has re-armed by staying silent at start."""
    arb = Arbiter(CFG, now=t)
    arb.tick(t + CFG.nav_timeout_sec + 0.01)
    assert arb.armed
    return arb


def rearm(arb: Arbiter, source: str, t: float) -> float:
    for i in range(CFG.rearm_neutral_count):
        assert arb.on_command(source, STILL, t + 0.01 * i) == ZERO
    return t + 0.01 * CFG.rearm_neutral_count


def test_starts_disarmed_and_never_forwards_a_held_command():
    arb = Arbiter(CFG, now=0.0)
    assert not arb.armed
    for i in range(20):
        assert arb.on_command(NAV, MOVE, 0.05 * i) == ZERO  # moving all the time: never armed
    assert not arb.armed


def test_navigation_rearms_after_silence_or_neutral_commands():
    assert armed_auto().on_command(NAV, MOVE, 1.0) == MOVE
    arb = Arbiter(CFG, now=0.0)
    rearm(arb, NAV, 0.1)
    assert arb.armed and arb.on_command(NAV, MOVE, 0.2) == MOVE


def test_only_the_active_source_is_forwarded():
    """Finding #8: one topic per remote source, only the selected one drives."""
    arb = armed_auto()
    assert arb.on_command('handheld', MOVE, 1.0) is None
    assert arb.set_mode('remote', 'dashboard', 1.0).changed
    assert arb.on_command(NAV, MOVE, 1.1) is None
    assert arb.on_command('handheld', MOVE, 1.1) is None
    t = rearm(arb, 'dashboard', 1.2)
    assert arb.on_command('dashboard', MOVE, t) == MOVE
    assert arb.on_command('handheld', MOVE, t) is None


def test_set_mode_same_mode_and_source_is_a_no_op():
    """Finding #13: re-sending the mode injects nothing and keeps the source armed."""
    arb = armed_auto()
    reply = arb.set_mode('auto', '', 2.0)
    assert reply.success and not reply.changed and arb.armed
    arb.set_mode('remote', '', 2.0)
    assert arb.remote_source == 'handheld'  # empty: the first configured source
    t = rearm(arb, 'handheld', 2.1)
    for source in ('', 'handheld'):
        reply = arb.set_mode('remote', source, t)
        assert reply.success and not reply.changed
    assert arb.armed and arb.on_command('handheld', MOVE, t) == MOVE


def test_unknown_mode_or_source_rejected():
    arb = armed_auto()
    assert not arb.set_mode('manual', '', 1.0).success
    assert not arb.set_mode('remote', 'joystick', 1.0).success
    assert arb.mode is Mode.AUTO and arb.armed


def test_switch_to_remote_ignores_a_stick_held_during_the_switch():
    arb = armed_auto()
    arb.set_mode('remote', 'handheld', 1.0)
    assert arb.on_command('handheld', MOVE, 1.05) == ZERO
    t = rearm(arb, 'handheld', 1.1)
    assert arb.on_command('handheld', MOVE, t) == MOVE


def test_watchdog_zeroes_once_per_silence():
    arb = armed_auto()
    assert arb.on_command(NAV, MOVE, 1.0) == MOVE
    assert not arb.tick(1.0 + CFG.nav_timeout_sec - 0.01)
    assert arb.tick(1.0 + CFG.nav_timeout_sec + 0.01)
    assert not arb.tick(5.0)
    arb.set_mode('remote', 'handheld', 5.0)
    assert not arb.tick(5.1)  # the switch itself sent the zero
    t = rearm(arb, 'handheld', 6.0)
    arb.on_command('handheld', MOVE, t)
    assert arb.tick(t + CFG.remote_timeout_sec + 0.01)


def test_estop_reasons_are_kept_oldest_first_without_duplicates():
    """Finding #14: a second reason is added, not overwritten."""
    arb = armed_auto()
    assert arb.engage('service')
    assert arb.engage('bumper')
    assert not arb.engage('service')
    assert arb.engage('  ') and arb.reasons == ['service', 'bumper', 'unspecified']
    assert arb.state().estop and not arb.state().armed


def test_estop_blocks_every_source_and_mode_switches_drive_nothing():
    arb = armed_auto()
    arb.engage('test')
    assert arb.on_command(NAV, MOVE, 1.0) is None
    assert arb.set_mode('remote', 'handheld', 1.1).changed  # allowed while engaged
    for i in range(10):
        assert arb.on_command('handheld', STILL, 1.2 + 0.01 * i) is None
    assert not arb.armed  # neutral commands during the e-stop do not re-arm
    assert arb.set_mode('auto', '', 2.0).changed
    arb.tick(10.0)
    assert not arb.armed and arb.on_command(NAV, MOVE, 10.0) is None


def test_release_requires_rearm_and_never_forwards_the_held_command():
    """Finding B3: release does not forward the next held stick or navigation command."""
    arb = armed_auto()
    arb.set_mode('remote', 'handheld', 1.0)
    t = rearm(arb, 'handheld', 1.0)
    arb.engage('service')
    reply = arb.release(t + 1.0)
    assert reply.success and reply.changed and not arb.engaged and not arb.armed
    assert arb.on_command('handheld', MOVE, t + 1.1) == ZERO
    assert arb.on_command('handheld', STILL, t + 1.2) == ZERO
    assert arb.on_command('handheld', MOVE, t + 1.3) == ZERO  # the neutral count restarts
    t = rearm(arb, 'handheld', t + 1.4)
    assert arb.on_command('handheld', MOVE, t) == MOVE


def test_release_in_auto_waits_for_navigation_silence():
    arb = armed_auto()
    arb.engage('service')
    arb.release(1.0)
    assert arb.on_command(NAV, MOVE, 1.1) == ZERO
    arb.tick(1.1 + CFG.nav_timeout_sec / 2)
    assert not arb.armed
    arb.tick(1.1 + CFG.nav_timeout_sec + 0.01)
    assert arb.armed


def test_release_disarms_every_source_not_only_the_active_one():
    """Switching after a release must not reach a source that was armed before the e-stop."""
    arb = armed_auto()
    arb.set_mode('remote', 'handheld', 1.0)
    rearm(arb, 'handheld', 1.0)
    arb.set_mode('auto', '', 2.0)
    arb.engage('x')
    arb.release(2.5)
    arb.set_mode('remote', 'handheld', 2.6)
    assert arb.on_command('handheld', MOVE, 2.7) == ZERO


def test_hardware_estop_engages_and_blocks_release():
    """Finding C2: the hardware e-stop input latches and cannot be released in software."""
    arb = armed_auto()
    assert arb.set_hardware(True, 1.0)
    assert arb.engaged and arb.reasons == [HARDWARE_REASON]
    reply = arb.release(1.0)
    assert not reply.success and arb.engaged
    assert not arb.set_hardware(True, 1.0)  # repeated pressed messages change nothing
    assert arb.set_hardware(False, 1.0) and arb.engaged  # releasing the button does not release
    assert arb.release(1.1).success and not arb.engaged


def test_silent_hardware_estop_engages_when_required():
    cfg = MuxConfig(hw_estop_timeout_sec=0.5)
    arb = Arbiter(cfg, now=0.0)
    arb.set_hardware(False, 0.3)
    assert not arb.tick(0.7) and not arb.engaged
    assert arb.tick(0.81) and arb.reasons == [HW_SILENT_REASON]
    assert not arb.tick(0.9)  # once
    arb.set_hardware(False, 1.0)
    assert arb.release(1.0).success
    optional = Arbiter(CFG, now=0.0)
    optional.tick(100.0)
    assert not optional.engaged  # 0: the input is optional


def test_release_when_not_engaged_is_harmless():
    arb = armed_auto()
    reply = arb.release(1.0)
    assert reply.success and not reply.changed and arb.armed


def test_config_validation():
    with pytest.raises(ConfigError):
        build(MuxConfig, {'remote_sources': ['hand held']})
    with pytest.raises(ConfigError):
        build(MuxConfig, {'remote_sources': ['a', 'a']})
    with pytest.raises(ConfigError):
        build(MuxConfig, {'mode': 'remote', 'remote_sources': []})
    with pytest.raises(ConfigError):
        build(MuxConfig, {'nav_timeout_sec': 0.0})
    with pytest.raises(ConfigError):
        build(MuxConfig, {'remote_timeout_sec': float('nan')})
    auto_only = build(MuxConfig, {'remote_sources': []})
    arb = Arbiter(auto_only, now=0.0)
    assert not arb.set_mode('remote', '', 0.0).success
