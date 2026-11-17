"""Docking: centring on the tag, approach, settle check, sit and charge hooks, retries, undock."""

from __future__ import annotations

from dataclasses import replace
import math

from dock_rig import TAG, DockRig, docked, goal_pose, tag_matrix, with_goal
import pytest

from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.docking.state import D
from x30_utils.config import ConfigError, build
from x30_utils.math import Pose2D

CFG = DockingConfig()


def at_goal(rig, config=CFG, tol=0.04):
    goal = goal_pose(config)
    return (
        abs(rig.pose.x - goal.x) < tol
        and abs(rig.pose.y - goal.y) < tol
        and abs(math.remainder(rig.pose.yaw - goal.yaw, 2 * math.pi)) < math.radians(5.0)
    )


def is_zero(c) -> bool:
    return c is not None and c.vx == c.vy == c.wz == 0.0


def test_omni_centres_first_then_docks_sits_and_charges():
    """D1: offset and turned away, the robot centres, walks in, settles, sits, charges."""
    rig = DockRig(Pose2D(1.2, 0.35, math.radians(15.0)))
    rig.charging_after = 2.0
    assert rig.docking.dock()[0]
    decision = rig.run(90.0, until=docked)
    assert decision.report.state is D.DOCKED, decision.report.message
    assert decision.report.charging and at_goal(rig)
    assert rig.hooks == ['sit', 'charge']
    states = rig.states()  # SEARCHING ends in the first tick: the tag is in view
    assert states == [D.CENTERING, D.APPROACHING, D.SETTLING, D.SITTING, D.CHARGING, D.DOCKED]
    assert rig.decisions[0].notes[0].startswith('SEARCHING')
    # centred before walking in: no forward motion while the offset was large
    first_walk = next(i for i, d in enumerate(rig.decisions) if d.report.state is D.APPROACHING)
    assert all(abs(c.vx) < 0.05 for c in rig.commands[: first_walk - 1] if c is not None)
    assert rig.docking.dock()[0] is False  # sitting on the dock


def test_commands_stop_once_seated():
    """D2: one zero when the robot stops on the dock, then nothing while hooks run."""
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    rig.charging_after = 0.0
    rig.docking.dock()
    rig.run(60.0, until=lambda d: d.report.state is D.SITTING)
    sitting = rig.decisions[-1]
    assert is_zero(sitting.command)
    rig.run(10.0, until=docked)
    later = [d.command for d in rig.decisions if d.report.state in (D.CHARGING, D.DOCKED)]
    assert later and all(c is None for c in later)


def test_diff_robot_docks_by_steering():
    """D3: a diff robot corrects the lateral offset while walking (it cannot slide)."""
    rig = DockRig(Pose2D(0.8, -0.2, math.radians(-10.0)), can_strafe=False)
    rig.charging_after = 0.0
    rig.docking.dock()
    decision = rig.run(120.0, until=docked)
    assert decision.report.state is D.DOCKED, decision.report.message
    assert at_goal(rig) and all(c.vy == 0.0 for c in rig.commands)


def test_backward_docking_with_a_rear_camera():
    """D4: backing in, the robot ends facing away from the tag at the goal distance."""
    config = with_goal(CFG, direction='backward', distance=0.6)
    rig = DockRig(Pose2D(1.4, 0.2, math.pi + math.radians(10.0)), config, rear_camera=True)
    rig.charging_after = 0.0
    rig.docking.dock()
    decision = rig.run(90.0, until=docked)
    assert decision.report.state is D.DOCKED, decision.report.message
    assert abs(rig.pose.x - (TAG.x - 0.6)) < 0.04 and abs(rig.pose.y) < 0.04
    assert abs(math.remainder(rig.pose.yaw - math.pi, 2 * math.pi)) < math.radians(5.0)
    assert min(c.vx for c in rig.commands) < -0.02  # it walked backward


def test_lateral_offset_goal():
    config = with_goal(CFG, lateral_offset=0.1)
    rig = DockRig(Pose2D(1.5, -0.1, 0.0), config)
    rig.charging_after = 0.0
    rig.docking.dock()
    assert rig.run(90.0, until=docked).report.state is D.DOCKED
    assert at_goal(rig, config)


def test_tag_never_in_view_fails_after_the_retries():
    """D5: no tag: every attempt times out, backing off between them, then FAILED."""
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    rig.visible = False
    rig.docking.dock()
    decision = rig.run(120.0, until=docked)
    assert decision.report.state is D.FAILED and 'not in view' in decision.report.message
    assert decision.report.attempt == CFG.retry.max_retries + 1
    assert rig.states().count(D.BACKING_OFF) == CFG.retry.max_retries
    assert is_zero(decision.command)


def test_tag_lost_close_to_the_dock_is_fine():
    """D6: inside blind_distance the estimate is frozen; the camera may lose the tag."""
    rig = DockRig(Pose2D(1.5, 0.05, 0.0))
    rig.camera_min_range = CFG.goal.distance + CFG.approach.blind_distance  # lost at the end
    rig.charging_after = 0.0
    rig.docking.dock()
    decision = rig.run(90.0, until=docked)
    assert decision.report.state is D.DOCKED, decision.report.message
    assert at_goal(rig)


def test_estop_and_remote_end_the_attempt():
    """D7: the e-stop (or REMOTE) fails docking at once with a zero command."""
    for flag in ('estop', 'remote'):
        rig = DockRig(Pose2D(1.2, 0.0, 0.0))
        rig.docking.dock()
        rig.run(30.0, until=lambda d: d.report.state is D.APPROACHING)
        setattr(rig, flag, True)
        decision = rig.tick()
        assert decision.report.state is D.FAILED and is_zero(decision.command)
        assert rig.tick().command is None  # one zero, then nothing


def test_disarmed_sends_neutral_and_freezes_the_clocks():
    """D8: while the mux has not armed navigation: neutral commands, pause reason, no timeout."""
    config = replace(CFG, supervision=replace(CFG.supervision, search_timeout_sec=1.0))
    rig = DockRig(Pose2D(1.2, 0.0, 0.0), config)
    rig.armed = False
    rig.docking.dock()
    rig.run(5.0)
    last = rig.decisions[-1]
    assert last.report.state is D.SEARCHING and last.report.pause_reasons == ('disarmed',)
    assert all(is_zero(c) for c in rig.commands)
    rig.armed = True
    rig.charging_after = 0.0
    assert rig.run(60.0, until=docked).report.state is D.DOCKED


def test_held_robot_backs_off_and_retries():
    """D9: commanded but not moving (the control field holds it): the attempt fails."""
    config = replace(CFG, supervision=replace(CFG.supervision, stall_timeout_sec=2.0))
    rig = DockRig(Pose2D(1.2, 0.0, 0.0), config)
    rig.docking.dock()
    rig.run(30.0, until=lambda d: d.report.state is D.APPROACHING)
    rig.moves = False
    decision = rig.run(10.0, until=lambda d: d.report.state is D.BACKING_OFF)
    assert decision.report.state is D.BACKING_OFF and 'held' in decision.report.message
    rig.moves, rig.charging_after = True, 0.0
    assert rig.run(90.0, until=docked).report.state is D.DOCKED
    assert rig.decisions[-1].report.attempt == 2


def test_sit_hook_failure_fails_and_charge_not_confirmed_fails():
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    rig.hook_answers['sit'] = (False, 'motor fault')
    rig.docking.dock()
    decision = rig.run(60.0, until=docked)
    assert decision.report.state is D.FAILED and 'motor fault' in decision.report.message
    assert rig.hooks == ['sit'] and not rig.docking.seated

    config = replace(CFG, charge=replace(CFG.charge, confirm_timeout_sec=2.0))
    rig = DockRig(Pose2D(1.5, 0.0, 0.0), config)  # the battery never charges
    rig.docking.dock()
    decision = rig.run(60.0, until=docked)
    assert decision.report.state is D.FAILED and 'charging' in decision.report.message
    assert rig.docking.seated and not rig.docking.dock()[0]  # undock first


def test_undock_stops_charging_stands_and_backs_away():
    """D10: from DOCKED: stop_charge, stand, back away undock.distance, IDLE."""
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    rig.charging_after = 0.0
    rig.docking.dock()
    assert rig.run(60.0, until=docked).report.state is D.DOCKED
    before = rig.pose
    assert not rig.docking.dock()[0]
    assert rig.docking.undock()[0]
    decision = rig.run(30.0, until=lambda d: d.report.state is D.IDLE)
    assert decision.report.state is D.IDLE and decision.report.message == 'undocked'
    assert rig.hooks == ['sit', 'charge', 'stop_charge', 'stand']
    assert before.x - rig.pose.x >= CFG.undock.distance - 0.01
    assert is_zero(decision.command) and not rig.docking.seated


def test_cancel_drops_late_hook_answers():
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    rig.docking.dock()
    rig.run(60.0, until=lambda d: d.hooks is not None)
    ticket = rig.decisions[-1].hooks.ticket
    rig.docking.cancel()
    assert not rig.docking.on_hooks(ticket, True, 'late')
    assert rig.tick().report.state is D.IDLE


def test_busy_refuses_new_requests():
    rig = DockRig(Pose2D(1.5, 0.0, 0.0))
    assert rig.docking.dock()[0]
    assert not rig.docking.dock()[0] and not rig.docking.undock()[0]


def test_config_validation():
    bad = {'centering.lateral_tolerance': 0.2, 'centering.realign_lateral': 0.1}
    with pytest.raises(ConfigError, match='realign_lateral'):
        build(DockingConfig, bad)
    assert tag_matrix(TAG).shape == (4, 4)
