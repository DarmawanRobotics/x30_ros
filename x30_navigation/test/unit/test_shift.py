"""Path shift: corridor in route coordinates, offset choice, return delay, speed limits."""

from __future__ import annotations

import math

import numpy as np
import pytest
from sim import OUT_AND_BACK, block, line, route_of, straight_route

from x30_navigation.core.config.shift import PathShiftConfig
from x30_navigation.core.shift.path_shift import HOLD_SPEED, PathShift
from x30_utils.robot import Footprint, Limits, RobotProfile
from x30_utils.safety import stopping_speed

ROBOT = RobotProfile(
    footprint=Footprint(front=0.40, rear=0.40, half_width=0.28),
    limits=Limits(max_decel=0.5, reaction_time=0.0),
)
CONFIG = PathShiftConfig()


def shifter(config=CONFIG, route=None):
    return PathShift(route or route_of(straight_route(0.0, 20.0)), ROBOT, config)


def person(x, y, r=0.25, n=12):
    """Points on the outline of someone standing at x, y."""
    return np.array([(x + r * math.cos(a), y + r * math.sin(a)) for a in np.linspace(0, 6.3, n)])


def choose(points, current=0.0, s_robot=5.0, config=CONFIG):
    """The robot at its current offset (target and committed) chooses for the points."""
    shift = shifter(config)
    shift.target = shift.offset = current
    s_pts, d_pts = shift.corridor.coordinates(s_robot, np.asarray(points, float))
    return shift, shift.choose(s_pts, d_pts, s_robot)


def test_clear_route_keeps_the_offset_at_zero():
    _, (offset, free, clear) = choose([(8.0, 2.5), (11.0, -2.0)])
    assert clear and offset == 0.0 and math.isinf(free)


def test_person_on_the_route_is_passed_with_clearance():
    _, (offset, _, clear) = choose(person(7.0, 0.0))
    assert clear and 0.25 + 0.28 + 0.10 <= abs(offset) <= 0.8 + 1e-9


def test_keeps_its_side_and_returns_only_when_past():
    _, keep = choose(person(7.0, 0.0), current=0.7)
    assert keep[0] == pytest.approx(0.7)
    _, beside = choose(person(5.0, 0.0), current=0.7)
    assert beside[0] == pytest.approx(0.7)
    _, behind = choose(person(3.8, 0.0), current=0.7)
    assert behind[0] == 0.0


def test_no_gap_slows_to_a_stop_with_the_stopping_model():
    """N3: the limit is x30_utils.safety.stopping_speed of the free distance."""
    wall = np.vstack([person(7.0, y) for y in np.arange(-1.4, 1.5, 0.4)])
    shift, (_, free, clear) = choose(wall)
    assert not clear
    limit = shift.speed_limit(free)
    assert limit == pytest.approx(stopping_speed(free - CONFIG.stop_margin, 0.5, 0.0))
    _, (_, close, _) = choose(wall, s_robot=6.2)
    assert shift.speed_limit(close) == 0.0


def test_return_waits_for_the_delay():
    shift = shifter()
    out = shift.update(5.0, 0.0, person(7.0, 0.0), False, 0.0, 0.05)
    side = out.target
    assert side != 0.0
    shift.offset = side
    assert shift.update(5.0, side, np.zeros((0, 2)), False, 1.0, 0.05).target == side
    assert shift.update(5.0, side, np.zeros((0, 2)), False, 1.6, 0.05).target == 0.0


def test_blind_scan_stops():
    """F1: a scan with blocked sectors gives speed limit 0, never a clear corridor."""
    out = shifter().update(5.0, 0.0, np.zeros((0, 2)), True, 0.0, 0.05)
    assert out.blind and out.speed_limit == 0.0 and not out.clear


def test_measured_offset_limits_the_speed():
    """F4: commanded offset clear but the robot is still beside the obstacle line: slow."""
    shift = shifter()
    shift.target = shift.offset = 0.7
    obstacle = person(6.2, 0.0)
    lagging = shift.update(5.0, 0.0, obstacle, False, 0.0, 0.05)
    there = shifter()
    there.target = there.offset = 0.7
    arrived = there.update(5.0, 0.7, obstacle, False, 0.0, 0.05)
    assert lagging.speed_limit < 0.45 and math.isinf(arrived.speed_limit)


def test_offset_never_exceeds_max_offset():
    box = [(7.0, y) for y in np.arange(-0.7, 0.71, 0.05)]
    _, (_, _, clear) = choose(box)
    assert not clear


def test_wall_behind_the_start_and_beyond_the_end_are_ignored():
    shift = shifter(route=route_of(straight_route(0.0, 10.0)))
    wall = np.array([(-1.5, y) for y in np.arange(-1.5, 1.51, 0.1)])
    s_pts, d_pts = shift.corridor.coordinates(0.0, wall)
    assert shift.choose(s_pts, d_pts, 0.0) == (0.0, math.inf, True)
    s_pts, d_pts = shift.corridor.coordinates(9.0, person(11.0, 0.0))
    assert shift.choose(s_pts, d_pts, 9.0)[0] == 0.0


def test_the_return_leg_is_not_in_the_way():
    """F5: on an out-and-back route the parallel return leg (0.8 m away) is the route too."""
    shift = shifter(route=route_of(OUT_AND_BACK))
    s_pts, d_pts = shift.corridor.coordinates(4.0, person(6.0, 0.8))
    offset, _, clear = shift.choose(s_pts, d_pts, 4.0)
    assert clear and offset == 0.0


def test_committed_offset_waits_for_a_held_robot():
    """E4 seed 16: a held robot's offset never runs off toward a target on the other side."""
    shift = shifter()
    shift.target = shift.offset = 0.7
    crossing = person(6.5, 0.4)
    for i in range(200):
        out = shift.update(5.0, 0.7, crossing, False, 0.05 * i, 0.05)
    assert out.target < 0.0
    assert abs(0.7 - out.offset) <= CONFIG.max_lead + 1e-9


def test_committed_offset_follows_the_robot():
    shift = shifter()
    lateral = 0.0
    for i in range(100):
        out = shift.update(5.0, lateral, person(7.5, 0.0), False, 0.05 * i, 0.05)
        assert abs(out.offset - lateral) <= CONFIG.max_lead + 1e-9
        lateral += 0.5 * (out.offset - lateral)
    assert out.offset == pytest.approx(out.target)


def test_held_forward_still_slides_toward_the_offset():
    """E4 seeds 3, 7: speed 0 ahead (within stop_margin) keeps the sideways slide."""
    shift = shifter()
    for i in range(5):
        out = shift.update(5.0, 0.0, person(6.0, 0.0), False, 0.05 * i, 0.05)
    assert out.speed_limit == 0.0 and out.target != 0.0
    assert out.slide * out.target > 0.0


def test_no_return_through_a_person_beside():
    """E4: the way back through a person beside the robot is never taken."""
    a = np.linspace(-math.pi, 0.0, 9)
    near_side = np.column_stack((6.35 + 0.25 * np.cos(a), -0.03 + 0.25 * np.sin(a)))
    shift = shifter()
    shift.target = shift.offset = -0.8
    s_pts, d_pts = shift.corridor.coordinates(6.62, near_side)
    offset, _, clear = shift.choose(s_pts, d_pts, 6.62, lateral=-0.77)
    assert clear and offset <= -0.6


def corner_route():
    """East along y = 0 to a corner at (4.7, 0), then north."""
    east = straight_route(0.0, 4.7)
    north = [(4.7, y) for y in np.arange(0.1, 3.0, 0.1)]
    return route_of(east + north)


def test_points_beyond_an_outer_corner_keep_their_distance():
    """Open item: a wall 0.8 m beyond a corner was projected onto it (d = -0.3 m at s = 4.7)."""
    shift = shifter(route=corner_route())
    wall = np.array([(5.5, y) for y in np.arange(-1.5, 1.51, 0.05)])
    s_pts, d_pts = shift.corridor.coordinates(3.9, wall)
    corner = np.abs(s_pts - 4.7) < 1e-6
    assert corner.any()
    assert np.all(np.abs(d_pts[corner]) >= 0.8 - 1e-6)
    assert np.all(d_pts[corner] < 0.0)  # the outer side of a left turn is on the right
    out = shift.update(3.9, 0.0, wall, False, 0.0, 0.05, hold=True)
    assert math.isinf(out.free_distance)


def test_a_point_at_the_outer_corner_still_blocks():
    shift = shifter(route=corner_route())
    s_pts, d_pts = shift.corridor.coordinates(3.9, np.array([[4.9, -0.1]]))
    assert s_pts == pytest.approx([4.7]) and d_pts == pytest.approx([-math.hypot(0.2, 0.1)])
    out = shift.update(3.9, 0.0, np.array([[4.9, -0.1]]), False, 0.0, 0.05, hold=True)
    assert out.free_distance == pytest.approx(4.7 - 3.9 - 0.4)


def wide_box(x=5.8, half=0.9):
    """A box 1.8 m wide across the route: no clear offset within max_offset (0.8 m)."""
    return np.array([(x, y) for y in np.arange(-half, half + 0.01, 0.05)])


def hold_for(shift, points, seconds, t=0.0, s_robot=5.0, lateral=0.0, **kwargs):
    """Update a robot standing at s_robot for seconds; return the last output and the time."""
    out = None
    for _ in range(round(seconds / 0.05)):
        out = shift.update(s_robot, lateral, points, False, t, 0.05, **kwargs)
        t += 0.05
    return out, t


def test_held_without_a_clear_offset_searches_after_the_patience():
    """Field (L1W): held patience_sec with no clear offset, path shift searches further."""
    shift = shifter()
    out, t = hold_for(shift, wide_box(), CONFIG.patience_sec - 0.2)  # 0.4 m ahead: held
    assert out.speed_limit == 0.0 and not out.clear and not out.searching
    assert abs(out.target) <= CONFIG.max_offset + 1e-9
    out, _ = hold_for(shift, wide_box(), 0.4, t)
    assert out.searching and out.clear
    assert 0.9 + 0.28 + 0.10 <= abs(out.target) <= CONFIG.search_offset + 1e-9
    assert out.slide * out.target > 0.0  # sliding toward it, forward still held


def test_the_guard_holding_the_robot_counts_as_waiting():
    """Far enough for path shift's limit, but the caller's guard holds it (held=True)."""
    shift = shifter()
    box = wide_box(x=6.0)
    out, _ = hold_for(shift, box, CONFIG.patience_sec + 0.2)
    assert out.speed_limit > HOLD_SPEED and not out.searching  # creeping on: not waiting
    out, _ = hold_for(shifter(), box, CONFIG.patience_sec + 0.2, held=True)
    assert out.searching and out.clear


def test_a_clear_offset_or_moving_on_restarts_the_patience():
    shift = shifter()
    _, t = hold_for(shift, wide_box(), CONFIG.patience_sec - 0.5)
    _, t = hold_for(shift, person(5.8, 0.0), 0.1, t)  # it became passable for a moment
    out, _ = hold_for(shift, wide_box(), CONFIG.patience_sec - 0.5, t)
    assert not out.searching


def test_the_search_ends_back_near_the_route():
    shift = shifter()
    out, t = hold_for(shift, wide_box(), CONFIG.patience_sec + 0.5)
    assert out.searching
    lateral, s_robot = out.target, 9.0  # it slid there and passed the box
    shift.offset = lateral
    for _ in range(120):
        out = shift.update(s_robot, lateral, np.zeros((0, 2)), False, t, 0.05)
        lateral, t = out.offset, t + 0.05  # the robot follows its committed offset
        if not out.searching:
            break
    assert not out.searching and out.target == 0.0
    assert abs(out.offset) <= CONFIG.max_offset + 1e-9


def test_near_a_stop_point_there_is_no_search():
    shift = shifter()
    out, t = hold_for(shift, wide_box(), CONFIG.patience_sec + 0.5)
    assert out.searching
    out, _ = hold_for(shift, wide_box(), 1.0, t, hold=True)
    assert not out.searching and out.target == 0.0


def test_search_offset_within_max_offset_never_searches():
    config = PathShiftConfig(search_offset=0.8)
    out, _ = hold_for(shifter(config), wide_box(), 10.0)
    assert not out.searching and not out.clear and abs(out.target) <= 0.8 + 1e-9


def test_an_emergency_holds_the_forward_motion_and_slides_on():
    """forward=False: speed limit 0 on a clear corridor; the slide toward the offset goes on."""
    shift = shifter()
    shift.target = shift.offset = 0.5
    out = shift.update(5.0, 0.0, np.zeros((0, 2)), False, 0.0, 0.05, forward=False)
    assert out.clear and out.speed_limit == 0.0 and out.slide > 0.0


def test_a_person_beside_does_not_hold_a_robot_with_nothing_ahead():
    """Field (L1W): points beside are passed parallel when nothing is ahead."""
    shift = shifter()
    side = np.vstack([person(5.0, 0.33 + 0.25), line(0.0, 20.0, -0.42)])
    out = shift.update(5.0, 0.0, side, False, 0.0, 0.05)
    assert out.clear and math.isinf(out.speed_limit) and out.target == pytest.approx(0.0)


@pytest.mark.parametrize('end', [5.3, 7.0])
def test_the_way_back_waits_while_the_robot_drives_on_parallel(end):
    """On the way back past benches 12 cm beside, it drives on and returns behind."""
    shift = shifter()
    shift.offset, shift.target = 0.6, 0.0  # on its way back
    benches = np.vstack([block(3.0, end, -0.4, 0.20), block(3.0, end, 1.0, 2.0)])
    out = shift.update(5.0, 0.6, benches, False, 0.0, 0.05)
    assert out.target == pytest.approx(0.6, abs=0.02) and math.isinf(out.speed_limit)
    assert shift.centred is (end > 5.4)
    t, lateral = 0.0, 0.6
    for _ in range(200):  # past them, nothing around: back onto the route
        out = shift.update(9.0, lateral, np.zeros((0, 2)), False, t, 0.05)
        lateral, t = out.offset, t + 0.05
    assert out.target == 0.0


def test_sliding_away_from_a_point_beside_is_always_reachable():
    """2 cm from a bench beside it, every offset away from it is reachable."""
    _, (offset, _, clear) = choose(block(3.0, 7.0, -0.4, 0.30), current=0.6)
    assert clear and 0.30 + 0.28 + 0.10 <= offset <= 0.8 + 1e-9


def test_a_narrow_corridor_is_driven_in_its_middle():
    """A 0.80 m corridor off the route: no 10 cm step has the clearance, its middle has."""
    shift = shifter()
    corridor = np.vstack([block(6.0, 10.0, 0.35, 2.5), block(6.0, 10.0, -2.5, -0.45)])
    out = shift.update(5.0, 0.0, corridor, False, 0.0, 0.05)
    assert out.clear and out.target == pytest.approx(-0.05, abs=1e-6)
    out = shift.update(7.0, -0.05, corridor, False, 0.05, 0.05)  # inside, in the middle
    assert out.target == pytest.approx(-0.05, abs=1e-6) and math.isinf(out.speed_limit)


def test_a_corridor_without_room_for_the_clearance_is_not_entered():
    shift = shifter()
    corridor = np.vstack([block(5.8, 10.0, 0.36, 2.5), block(5.8, 10.0, -2.5, -0.36)])
    out = shift.update(5.0, 0.0, corridor, False, 0.0, 0.05)  # 0.4 m before it
    assert not out.clear and out.speed_limit == 0.0


def test_a_wide_lane_keeps_the_route():
    shift = shifter()
    hall = np.vstack([block(6.0, 10.0, 1.0, 2.5), block(6.0, 10.0, -2.5, -0.6)])
    out = shift.update(5.0, 0.0, hall, False, 0.0, 0.05)
    assert out.clear and out.target == 0.0 and not shift.centred


def test_a_target_off_the_grid_keeps_its_clearance_alone():
    """Leaving a lane's middle between walls keeps the offset with the clearance alone."""
    walls = np.vstack([line(3.0, 5.4, 0.35), line(3.0, 5.4, -0.45)])  # beside, ending here
    _, (offset, _, clear) = choose(walls, current=-0.05)
    assert clear and offset == pytest.approx(-0.05)
