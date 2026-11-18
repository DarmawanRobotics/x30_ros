"""Avoidance policies, blind sectors, asymmetric smoothing and the sector scan helpers."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_navigation.core.avoidance import POLICIES, AvoidanceOut, AvoidanceShared
from x30_navigation.core.avoidance.smoothing import AsymmetricSmoother
from x30_navigation.core.scan import SectorScanData, body_extent
from x30_utils.robot import Footprint, Limits, RobotProfile

N, ANGLE_MIN = 32, -math.pi / 2
INC = math.pi / N
NAMES = ['braitenberg', 'follow_gap', 'potential_field', 'vfh']
SHARED = AvoidanceShared(safe_distance=1.5, emergency_distance=0.75, smoothing=0.0)


def scan(fn) -> SectorScanData:
    """Front scan from a function of the sector angle in degrees."""
    ranges = [fn(math.degrees(ANGLE_MIN + (i + 0.5) * INC)) for i in range(N)]
    return SectorScanData.of(ANGLE_MIN, INC, 8.0, ranges)


SMALL = RobotProfile(
    kinematics='diff', footprint=Footprint(0.2, 0.2, 0.15), limits=Limits(max_vy=0.0)
)


def make(name, shared=SHARED, robot=SMALL):
    return POLICIES.create(name, shared, robot=robot)


def test_every_builtin_is_registered():
    assert POLICIES.names() == NAMES


@pytest.mark.parametrize('name', NAMES)
def test_clear_path_is_untouched(name):
    out = make(name).update(scan(lambda a: 8.0))
    assert abs(out.heading_offset) < 1e-6 and out.velocity_scale == pytest.approx(1.0)
    assert not out.emergency


@pytest.mark.parametrize('name', NAMES)
def test_wall_ahead_is_an_emergency(name):
    assert make(name).update(scan(lambda a: 0.6)).emergency


@pytest.mark.parametrize('name', NAMES)
@pytest.mark.parametrize('blind', [0.0, math.nan, -1.0])
def test_blind_sectors_ahead_stop_the_robot(name, blind):
    """F1: a blind sensor (0 / NaN sectors) reads as an obstacle at the robot, not as free."""
    out = make(name).update(scan(lambda a: blind))
    assert out.emergency and out.velocity_scale == 0.0
    partly = make(name).update(scan(lambda a: blind if abs(a) < 10 else 8.0))
    assert partly.emergency


@pytest.mark.parametrize('name', ['follow_gap', 'potential_field', 'vfh'])
def test_obstacle_left_turns_right(name):
    """N4: heading_offset is in radians, + left; an obstacle left of the heading gives < 0."""
    out = make(name).update(scan(lambda a: 1.0 if 0 < a < 20 else 8.0))
    assert -math.pi / 2 < out.heading_offset < 0.0


def test_heading_offset_is_limited():
    shared = AvoidanceShared(safe_distance=1.5, smoothing=0.0, max_heading_offset_deg=10.0)
    out = make('vfh', shared).update(scan(lambda a: 1.0 if -40 < a < 60 else 8.0))
    assert abs(out.heading_offset) <= math.radians(10.0) + 1e-9


def test_smoothing_never_slows_braking():
    """F13: velocity_scale drops at once and only rises through the filter."""
    smoother = AsymmetricSmoother(0.9)
    smoother.step(AvoidanceOut(velocity_scale=1.0))
    assert smoother.step(AvoidanceOut(velocity_scale=0.1)).velocity_scale == pytest.approx(0.1)
    rising = smoother.step(AvoidanceOut(velocity_scale=1.0)).velocity_scale
    assert 0.1 < rising < 0.3
    assert smoother.step(AvoidanceOut(emergency=True)).emergency
    steering = smoother.step(AvoidanceOut(heading_offset=1.0)).heading_offset
    assert 0.0 < steering < 0.2


def test_policy_brakes_in_one_scan_with_heavy_smoothing():
    """F13: with smoothing 0.95 a person stepping in front slows the robot immediately."""
    policy = make('vfh', AvoidanceShared(safe_distance=1.5, smoothing=0.95))
    for _ in range(20):
        policy.update(scan(lambda a: 8.0))
    out = policy.update(scan(lambda a: 0.9 if abs(a) < 15 else 8.0))
    assert out.velocity_scale == pytest.approx(1.0 - (1.0 - 0.9 / 1.5))


def test_validate_distances():
    assert AvoidanceShared(safe_distance=0.5, emergency_distance=0.75).validate()


def test_sector_points_include_blocked_and_edges():
    data = SectorScanData.of(-math.pi, 2 * math.pi / 8, 5.0, [1.0, 5.0, 0.0, math.nan] + [5.0] * 4)
    pts = data.points()
    assert pts.full_circle and pts.blocked.tolist()[:4] == [False, False, True, True]
    assert len(pts.points) == 3 and len(pts.with_blocked()) == 5
    assert data.cleaned().tolist()[:4] == [1.0, 5.0, 0.0, 0.0]


def test_body_extent():
    fp = Footprint(front=0.4, rear=0.3, half_width=0.25)
    ext = body_extent(fp, np.array([0.0, math.pi / 2, math.pi]))
    assert ext == pytest.approx([0.4, 0.25, 0.3])


ROBOT = RobotProfile(kinematics='diff', limits=Limits(max_vy=0.0))  # half_width 0.28 m
MARGIN = 0.10  # body_margin of these geometry checks (the default is wider, see test_reactive)


def person_scan(x, y, r=0.25, n=N, angle_min=ANGLE_MIN, inc=INC) -> SectorScanData:
    """Sector scan (nearest range per sector) of someone standing at x, y."""
    ranges = np.full(n, 8.0)
    for a in np.linspace(0.0, 2.0 * math.pi, 180, endpoint=False):
        px, py = x + r * math.cos(a), y + r * math.sin(a)
        i = int(np.floor((math.atan2(py, px) - angle_min) / inc))
        if 0 <= i < n:
            ranges[i] = min(ranges[i], math.hypot(px, py))
    return SectorScanData.of(angle_min, inc, 8.0, ranges)


def course_clearance(heading: float, scan: SectorScanData, horizon: float) -> float:
    """Smallest distance of the scan points (sector centres) from the straight course."""
    pts = scan.points(edges=False).points
    along = pts[:, 0] * math.cos(heading) + pts[:, 1] * math.sin(heading)
    across = np.abs(-pts[:, 0] * math.sin(heading) + pts[:, 1] * math.cos(heading))
    ahead = (along > 0.0) & (np.hypot(pts[:, 0], pts[:, 1]) < horizon)
    return float(across[ahead].min()) if ahead.any() else math.inf


@pytest.mark.parametrize('name', ['vfh', 'follow_gap'])
@pytest.mark.parametrize('y', [-0.1, 0.0, 0.15])
def test_chosen_heading_is_a_collision_free_course(name, y):
    """E4: the chosen heading keeps half width plus body_margin from every obstacle."""
    shared = AvoidanceShared(
        safe_distance=1.5, emergency_distance=0.5, smoothing=0.0, body_margin=MARGIN
    )
    scan_ = person_scan(1.0, y)
    out = make(name, shared, ROBOT).update(scan_)
    hw = ROBOT.footprint.half_width
    assert out.velocity_scale > 0.0
    assert course_clearance(out.heading_offset, scan_, 1.5) >= hw + MARGIN - 0.03
    assert abs(out.heading_offset) > math.radians(14.0)


def test_omni_robot_sweeps_its_footprint_on_a_diagonal():
    """An omni robot keeps its yaw: moving at 90 deg it sweeps its length, not its width."""
    omni = make('vfh', robot=RobotProfile())
    extent = omni.half_extent(np.array([0.0, math.pi / 2, math.pi / 4]))
    fp = RobotProfile().footprint
    assert extent[:2] == pytest.approx([fp.half_width, fp.front])
    assert extent[2] > fp.half_width
    assert make('vfh', robot=ROBOT).half_extent(np.array([math.pi / 4])) == pytest.approx(0.28)


def test_vfh_inflation_grows_as_the_obstacle_comes_closer():
    shared = AvoidanceShared(
        safe_distance=2.5, emergency_distance=0.5, smoothing=0.0, body_margin=MARGIN
    )
    far = abs(make('vfh', shared, ROBOT).update(person_scan(2.0, 0.0)).heading_offset)
    near = abs(make('vfh', shared, ROBOT).update(person_scan(1.0, 0.0)).heading_offset)
    assert near > far > 0.0
