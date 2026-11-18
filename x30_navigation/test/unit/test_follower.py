"""The follower state machine: routes, stop points, joining, transitions and zero commands."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rig import Rig, S, is_zero, person
from sim import LOOP, OUT_AND_BACK, block, line, route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig, SafetyConfig
from x30_navigation.core.config.shift import DetourConfig
from x30_navigation.core.follower.state import ALLOWED, TransitionError, check_transition
from x30_utils.math import Pose2D


def test_transition_table():
    """F18: every state can stop; finished runs only restart; nothing jumps into DWELLING."""
    for state in S:
        assert S.INACTIVE in ALLOWED[state] and S.FAILED in ALLOWED[state]
    with pytest.raises(TransitionError):
        check_transition(S.GOAL_REACHED, S.DWELLING)
    with pytest.raises(TransitionError):
        check_transition(S.INACTIVE, S.DWELLING)


def test_follows_to_the_goal_and_zeroes_once():
    """F2/F18: on reaching the goal one zero is published, then nothing."""
    rig = Rig(route_of(straight_route(0.0, 6.0)))
    assert rig.follower.start(rig.pose)[0]
    decision = rig.run(60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert decision.report.state is S.GOAL_REACHED and is_zero(decision.command)
    assert math.hypot(rig.pose.x - 6.0, rig.pose.y) < 0.06
    assert rig.tick().command is None
    states = [d.report.state for d in rig.decisions]
    assert S.FOLLOWING in states and S.APPROACHING in states


def test_stop_points_arrive_inspect_depart():
    route = route_of(straight_route(0.0, 6.0), stops={30: 1.0})
    rig = Rig(route)
    rig.follower.start(rig.pose)
    decision = rig.run(30.0, until=lambda d: d.inspect is not None)
    index, round_id = decision.inspect
    assert index == 30 and rig.state is S.DWELLING
    assert abs(rig.pose.x - 3.0) < 0.06
    events = rig.follower.inspection_started(round_id, 1, [])
    assert events == []
    events = rig.follower.inspection_answer(round_id, 'camera', True, 'clear')
    assert [e.kind.name for e in events] == ['INSPECTED'] and 'camera: ok' in events[0].message
    rig.run(60.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.events() == [('ARRIVED', 30), ('DEPARTED', 30)]


def test_dwell_timer_is_frozen_while_paused():
    """F7: time paused at a stop point does not count as dwelling."""
    rig = Rig(route_of(straight_route(0.0, 6.0), stops={30: 2.0}))
    rig.follower.start(rig.pose)
    rig.run(30.0, until=lambda d: d.report.state is S.DWELLING)
    rig.follower.pause()
    rig.run(5.0)
    assert rig.state is S.PAUSED and all(is_zero(c) for c in rig.commands[-50:])
    rig.follower.resume()
    rig.run(1.0)
    assert rig.state is S.DWELLING


def test_late_inspection_answers_are_dropped_after_a_restart():
    """F17: a restarted route forgets the pending inspections of the old stop."""
    route = route_of(straight_route(0.0, 6.0), stops={30: 1.0})
    rig = Rig(route)
    rig.follower.start(rig.pose)
    _, round_id = rig.run(30.0, until=lambda d: d.inspect is not None).inspect
    rig.follower.inspection_started(round_id, 1, [])
    rig.follower.set_route(route, rig.pose)
    assert rig.follower.inspection.pending == 0
    assert rig.follower.inspection_answer(round_id, 'camera', True, 'late') == []


def test_unavailable_inspectors_are_reported_at_once():
    rig = Rig(route_of(straight_route(0.0, 6.0), stops={30: 1.0}))
    rig.follower.start(rig.pose)
    _, round_id = rig.run(30.0, until=lambda d: d.inspect is not None).inspect
    events = rig.follower.inspection_started(round_id, 0, ['camera'])
    assert events[0].kind.name == 'INSPECTED' and 'camera: not available' in events[0].message


def test_loop_route_starts_the_next_lap_unless_the_battery_is_low():
    config = FollowerConfig(require_localization=False, loop_route=True)
    rig = Rig(route_of(LOOP, spacing=0.1), config)
    rig.follower.start(rig.pose)
    rig.run(120.0, until=lambda d: rig.follower.lap == 1)
    assert rig.follower.lap == 1 and rig.state is S.FOLLOWING
    rig.follower.on_battery(0.15)
    rig.run(120.0, until=lambda d: d.report.state is S.GOAL_REACHED)
    assert rig.state is S.GOAL_REACHED and rig.follower.lap == 1


def test_start_beside_the_route_joins_far_away_is_refused():
    route = route_of(straight_route(0.0, 6.0))
    rig = Rig(route, pose=Pose2D(3.0, 1.5, 0.0))
    ok, message = rig.follower.start(rig.pose)
    assert ok and 'joining' in message
    rig.run(30.0, until=lambda d: d.report.state is S.FOLLOWING)
    assert rig.state is S.FOLLOWING and abs(rig.pose.y) < 0.2
    far = Rig(route, pose=Pose2D(3.0, 8.0, 0.0))
    ok, message = far.follower.start(far.pose)
    assert not ok and 'from the route' in message


def test_stop_zeroes_once_and_start_resumes():
    """F2: navigation/stop publishes one zero; INACTIVE publishes nothing after that."""
    rig = Rig(route_of(straight_route(0.0, 6.0)))
    rig.follower.start(rig.pose)
    rig.run(3.0)
    assert rig.commands[-1].vx > 0.1
    assert is_zero(rig.follower.stop())
    assert rig.tick().command is None and rig.state is S.INACTIVE
    x = rig.pose.x
    ok, message = rig.follower.start(rig.pose)
    assert ok and message == 'on the route at point ' + str(round(float(x) * 10))


def test_overlapping_legs_keep_progress_and_stops():
    """F5/F6: out and back, progress stays on its leg, the return-leg stop waits."""
    route = route_of(OUT_AND_BACK, spacing=0.1)
    back = int(np.argmin(np.hypot(route.xy[:, 0] - 4.0, route.xy[:, 1] - 0.8)))
    stops = dict.fromkeys([back], 1.0)
    route = route_of([(p[0], p[1]) for p in route.xy], stops=stops)
    rig = Rig(route)
    rig.follower.start(rig.pose)
    rig.obstacles = person(4.0, 0.0)
    rig.run(25.0, until=lambda d: rig.pose.x > 8.0)
    assert rig.pose.x > 8.0 and rig.follower.tracker.progress.s < route.length / 2
    assert ('ARRIVED', back) not in rig.events()
    rig.obstacles = np.zeros((0, 2))
    rig.run(60.0, until=lambda d: d.inspect is not None)
    assert rig.events()[0] == ('ARRIVED', back)


def test_stop_point_with_an_obstacle_waits_instead_of_passing():
    """F6: near a stop point the robot does not slide around an obstacle on it: it waits."""
    route = route_of(straight_route(0.0, 8.0), stops={40: 1.0})
    rig = Rig(route)
    rig.follower.start(rig.pose)
    rig.obstacles = person(4.4, 0.0)
    rig.run(25.0)
    assert ('SKIPPED', 40) not in rig.events() and rig.pose.x < 4.0
    assert abs(rig.pose.y) < 0.15
    rig.obstacles = np.zeros((0, 2))
    rig.run(20.0, until=lambda d: d.inspect is not None)
    assert rig.events()[0] == ('ARRIVED', 40)


def test_mode_change_drops_the_path_shift():
    """F15: switching avoidance_mode away from path_shift puts the robot back on the route."""
    rig = Rig(route_of(straight_route(0.0, 10.0)))
    rig.follower.start(rig.pose)
    rig.obstacles = person(3.0, 0.0)
    rig.run(8.0, until=lambda d: abs(rig.follower.tracker.offset) > 0.3)
    assert abs(rig.follower.tracker.offset) > 0.3
    s = rig.follower.tracker.progress.s
    rig.follower.configure(FollowerConfig(require_localization=False, avoidance_mode='reactive'))
    assert rig.follower.tracker.offset == 0.0 and rig.follower.tracker.shift.target == 0.0
    assert rig.follower.tracker.progress.s == pytest.approx(s)


def test_robot_stopped_close_to_a_person_slides_past_it():
    """E4 seeds 3, 7: stopped within stop_margin before reaching the offset: not BLOCKED."""
    rig = Rig(route_of(straight_route(0.0, 12.0)), pose=Pose2D(5.0, 0.0, 0.0))
    rig.obstacles = person(6.0, 0.0)
    assert rig.follower.start(rig.pose)[0]
    rig.run(25.0)
    assert rig.pose.x > 7.0
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_guard_held_shift_still_slides_past():
    """The guard holds the diagonal first: the robot slides instead of standing BLOCKED."""
    safety = SafetyConfig(human_speed=1.0, blocked_timeout_sec=5.0)
    config = FollowerConfig(require_localization=False, safety=safety)
    rig = Rig(route_of(straight_route(0.0, 12.0)), config, pose=Pose2D(4.5, 0.0, 0.0))
    rig.obstacles = person(6.2, 0.0)
    assert rig.follower.start(rig.pose)[0]
    rig.run(30.0)
    assert rig.pose.x > 7.5
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_a_box_wider_than_max_offset_is_passed_after_the_patience():
    """Field (L1W): a box wider than max_offset is passed after the patience and search."""
    config = FollowerConfig(require_localization=False, detour=DetourConfig(enabled=False))
    rig = Rig(route_of(straight_route(0.0, 14.0)), config)
    rig.obstacles = np.array([(6.0, y) for y in np.arange(-0.9, 0.91, 0.05)])
    assert rig.follower.start(rig.pose)[0]
    rig.run(45.0)
    shift = rig.follower.config.path_shift
    offsets = [d.debug.get('offset', 0.0) for d in rig.decisions]
    assert shift.max_offset < max(map(abs, offsets)) <= shift.search_offset + 1e-9
    assert rig.pose.x > 9.0 and abs(rig.pose.y) < 0.1
    assert not rig.follower.tracker.shift.searching
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)
    assert any('searching' in d.report.message for d in rig.decisions)


def test_a_person_beside_and_a_wall_are_passed_parallel():
    """Field (L1W): a person 5 cm and a wall 14 cm beside are passed parallel."""
    rig = Rig(route_of(straight_route(0.0, 12.0)), pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = np.vstack([person(4.0, 0.33 + 0.25), line(0.0, 12.0, -0.42)])
    assert rig.follower.start(rig.pose)[0]
    rig.run(10.0)
    assert rig.pose.x > 7.0 and abs(rig.pose.y) < 0.01
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_a_narrow_corridor_off_the_route_is_driven_in_its_middle():
    """A 0.80 m corridor 5 cm right of the route is driven in its middle, not BLOCKED."""
    rig = Rig(route_of(straight_route(0.0, 14.0)))
    rig.obstacles = np.vstack([block(5.0, 9.0, 0.35, 2.5), block(5.0, 9.0, -2.5, -0.45)])
    assert rig.follower.start(rig.pose)[0]
    inside = []
    for _ in range(int(40.0 / rig.dt)):
        rig.tick()
        if 4.6 <= rig.pose.x <= 9.4:  # the body between the walls
            inside.append(rig.pose.y)
    assert inside and max(abs(y + 0.05) for y in inside) < 0.015
    assert rig.pose.x > 13.0 and abs(rig.pose.y) < 0.05
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_the_way_back_past_benches_drives_on_parallel():
    """On the way back, benches 12 cm beside: it drives on and returns behind them."""
    rig = Rig(route_of(straight_route(0.0, 14.0)), pose=Pose2D(4.0, 0.6, 0.0))
    rig.obstacles = np.vstack([block(3.0, 6.0, -0.4, 0.20), block(3.0, 7.0, 1.0, 2.0)])
    assert rig.follower.start(rig.pose)[0]
    rig.run(16.0)
    assert rig.pose.x > 9.0 and abs(rig.pose.y) < 0.1
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)


def test_stop_point_at_a_corner_facing_a_wall_is_reached():
    """Open item: a stop at a corner facing a wall 0.8 m ahead held the robot at x = 3.9."""
    from x30_utils.io import RoutePoint

    east = [RoutePoint(x, y) for x, y in straight_route(0.0, 4.7)]
    east[-1] = RoutePoint(4.7, 0.0, 0.0, 1.0)
    north = [RoutePoint(4.7, float(y), math.pi / 2) for y in np.arange(0.1, 3.0, 0.1)]
    rig = Rig(route_of(east + north))
    rig.obstacles = np.array([(5.5, y) for y in np.arange(-2.0, 3.5, 0.05)])
    assert rig.follower.start(rig.pose)[0]
    rig.run(40.0, until=lambda d: d.inspect is not None)
    assert rig.events()[:1] == [('ARRIVED', len(east) - 1)]
    assert rig.pose.x == pytest.approx(4.7, abs=0.06)
    assert all(d.report.state is not S.BLOCKED for d in rig.decisions)
