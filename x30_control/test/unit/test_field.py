from __future__ import annotations

import math

import numpy as np
import pytest

from x30_control.core.command import ZERO, Command
from x30_control.core.config import FieldConfig
from x30_control.core.field import FieldState, ProtectiveField
from x30_control.core.scan import ScanState, ScanView
from x30_utils.robot import RobotProfile
from x30_utils.safety import ArcCheck, arc_poses, rotation_room, stopping_distance

ROBOT = RobotProfile()
CFG = FieldConfig(human_speed=0.0)  # the braking model alone; the person term: test_separation
FIELD = ProtectiveField(ROBOT, CFG)
LIM = ROBOT.limits


def ok(points) -> ScanView:
    return ScanView(ScanState.OK, np.asarray(points, dtype=float).reshape(-1, 2))


def wall(x0: float, y_from: float = -2.0, y_to: float = 2.0, step: float = 0.05):
    n = round((y_to - y_from) / step) + 1
    return [(x0, y_from + i * step) for i in range(n)]


def auto(cmd: Command, points):
    return FIELD.limit(cmd, ok(points), remote=False)


def test_free_path_is_clear_in_every_direction():
    for cmd in (Command(0.6, 0.0, 0.0), Command(0.0, 0.4, 0.0), Command(-0.3, 0.0, 0.8)):
        result = auto(cmd, [])
        assert result.state is FieldState.CLEAR and result.command == cmd
        assert math.isinf(result.clearance) and result.scale == 1.0


def test_wall_ahead_limits_with_the_shared_stopping_model():
    """Finding N3: the allowed speed stops exactly margin short (x30_utils.safety model)."""
    result = auto(Command(0.8, 0.0, 0.0), wall(1.2))
    assert result.state is FieldState.LIMITED and 0.0 < result.scale < 1.0
    assert result.clearance == pytest.approx(0.8)
    v = result.command.vx
    assert stopping_distance(v, LIM.max_decel, LIM.reaction_time) == pytest.approx(
        0.8 - CFG.margin
    )


def test_wall_close_stops_forward_but_not_backing_away_or_sideways():
    assert auto(Command(0.3, 0.0, 0.0), wall(0.5)).state is FieldState.STOP
    assert auto(Command(0.3, 0.0, 0.0), wall(0.5)).command == ZERO
    assert auto(Command(-0.3, 0.0, 0.0), wall(0.5)).state is FieldState.CLEAR
    assert auto(Command(0.0, 0.3, 0.0), wall(0.6, -0.2, 0.2)).state is FieldState.CLEAR


def test_reverse_and_sideways_are_protected():
    assert auto(Command(-0.3, 0.0, 0.0), [(-0.6, 0.0)]).state is FieldState.LIMITED
    assert auto(Command(-0.3, 0.0, 0.0), [(-0.54, 0.0)]).state is FieldState.STOP
    assert auto(Command(0.0, 0.3, 0.0), [(0.0, 0.42)]).state is FieldState.STOP
    assert auto(Command(0.3, 0.0, 0.0), [(0.0, 0.42)]).state is FieldState.CLEAR


def test_point_inside_the_footprint_stops_motion_toward_it():
    assert auto(Command(0.2, 0.0, 0.0), [(0.3, 0.0)]).command == ZERO


def test_rotation_next_to_an_obstacle_is_limited():
    """Finding C1: turning in place sweeps the corners into something beside the robot."""
    beside = [(0.25, 0.4)]  # inside the swept ring, outside the footprint
    turn = auto(Command(0.0, 0.0, 1.0), beside)
    assert turn.state in (FieldState.LIMITED, FieldState.STOP)
    assert abs(turn.command.wz) < 1.0
    touching = [(0.3, 0.3)]  # by the front-left corner: a left turn hits it within 4 deg
    assert auto(Command(0.0, 0.0, 0.5), touching).command.wz == 0.0
    assert auto(Command(0.0, 0.0, 1.0), [(2.0, 0.0)]).state is FieldState.CLEAR  # outside


def test_rotation_limit_follows_the_yaw_stopping_model():
    beside = [(0.25, 0.4)]
    wz = auto(Command(0.0, 0.0, 1.0), beside).command.wz
    assert 0.0 < wz < 1.0
    room = rotation_room(ROBOT.footprint, np.array(beside), 1.0) - CFG.yaw_margin
    assert stopping_distance(wz, LIM.max_yaw_accel, LIM.reaction_time) == pytest.approx(room)


def test_turning_away_from_a_corner_obstacle_is_allowed():
    """The front-left corner moves away from a point ahead-left when turning right."""
    point = [(0.3, 0.3)]
    assert auto(Command(0.0, 0.0, -0.5), point).command.wz == pytest.approx(-0.5)


def test_combined_motion_checks_the_arc():
    """An obstacle off the straight path and outside the turning ring is on the curved path."""
    obstacle = [(1.0, 0.6)]  # 1 m radius left arc passes through it
    assert auto(Command(0.8, 0.0, 0.0), obstacle).state is FieldState.CLEAR
    assert auto(Command(0.0, 0.0, 0.8), obstacle).state is FieldState.CLEAR
    result = auto(Command(0.8, 0.0, 0.8), obstacle)
    assert result.state in (FieldState.LIMITED, FieldState.STOP)
    out = result.command
    assert out.vx < 0.8 and out.wz == pytest.approx(out.vx)  # curvature kept
    # the limited command stops short of the obstacle along its own arc
    arc = ArcCheck(ROBOT.footprint, LIM, CFG.lateral_margin)
    assert arc.scale(out, np.array(obstacle), CFG.margin, CFG.yaw_margin) == pytest.approx(1.0)
    assert auto(Command(0.8, 0.0, -0.8), obstacle).state is FieldState.CLEAR  # turning away


def test_arc_poses_match_the_constant_twist():
    poses = arc_poses(Command(1.0, 0.0, 1.0), np.array([math.pi / 2]))
    assert poses[0] == pytest.approx([1.0, 1.0, math.pi / 2])
    straight = arc_poses(Command(0.5, 0.2, 0.0), np.array([2.0]))
    assert straight[0] == pytest.approx([1.0, 0.4, 0.0])


def test_arc_scale_is_one_for_pure_motions():
    arc = ArcCheck(ROBOT.footprint, LIM, CFG.lateral_margin)
    assert arc.scale(Command(0.5, 0.0, 0.0), np.array([[0.6, 0.0]]), 0.1, 0.1) == 1.0
    assert arc.scale(Command(0.0, 0.0, 0.5), np.array([[0.0, 0.45]]), 0.1, 0.1) == 1.0


def test_blind_scan_stops_auto():
    """Finding B1: blocked sectors are obstacles at the robot: AUTO does not move."""
    view = ScanView(ScanState.BLIND, np.empty((0, 2)), blocked=72)
    result = FIELD.limit(Command(0.4, 0.0, 0.3), view, remote=False)
    assert result.command == ZERO and result.state is FieldState.BLIND


def test_stale_scan_stops_auto_and_caps_remote():
    view = ScanView(ScanState.STALE, np.empty((0, 2)))
    assert FIELD.limit(Command(0.4, 0.0, 0.0), view, remote=False).command == ZERO
    result = FIELD.limit(Command(0.6, 0.0, 1.0), view, remote=True)
    assert result.state is FieldState.STALE
    out = result.command
    fastest = out.speed + abs(out.wz) * ROBOT.footprint.circumscribed_radius
    assert fastest == pytest.approx(CFG.remote_degraded_speed)
    assert out.wz / out.vx == pytest.approx(1.0 / 0.6)
    slow = Command(0.05, 0.0, 0.0)
    assert FIELD.limit(slow, view, remote=True).command == slow


def test_blind_remote_creeps_and_still_respects_seen_obstacles():
    view = ScanView(ScanState.BLIND, np.array(wall(0.5)), blocked=3)
    result = FIELD.limit(Command(0.5, 0.0, 0.0), view, remote=True)
    assert result.state is FieldState.BLIND and result.command == ZERO
    away = FIELD.limit(Command(-0.5, 0.0, 0.0), view, remote=True)
    assert away.command.vx == pytest.approx(-CFG.remote_degraded_speed)


def test_remote_uses_its_margin_and_can_be_switched_off():
    """Finding N3: the field is on in REMOTE by default, with remote_margin."""
    cmd = Command(0.8, 0.0, 0.0)
    remote = FIELD.limit(cmd, ok(wall(1.2)), remote=True)
    v = remote.command.vx
    room = 0.8 - CFG.remote_margin
    assert stopping_distance(v, LIM.max_decel, LIM.reaction_time) == pytest.approx(room)
    off = ProtectiveField(ROBOT, FieldConfig(in_remote=False))
    result = off.limit(cmd, ok(wall(1.2)), remote=True)
    assert result.state is FieldState.OFF and result.command == cmd
    assert result.clearance == pytest.approx(0.8)


def test_min_speed_turns_a_crawl_into_a_stop():
    cfg = FieldConfig(min_speed=0.1)
    field = ProtectiveField(ROBOT, cfg)
    result = field.limit(Command(0.5, 0.0, 0.0), ok(wall(0.4 + CFG.margin + 0.02)), remote=False)
    assert result.command == ZERO and result.state is FieldState.STOP


def test_partly_blind_remote_does_not_translate_toward_blocked_sectors():
    """A blocked sector is an obstacle at the robot: REMOTE may move away from it, not toward."""
    width = 2 * math.pi / 72
    view = ScanView(ScanState.BLIND, np.empty((0, 2)), 1, 'x', np.array([0.0]), width)
    assert FIELD.limit(Command(0.3, 0.0, 0.0), view, remote=True).command.speed == 0.0
    assert FIELD.limit(Command(0.3, 0.3, 0.0), view, remote=True).command.speed == 0.0
    back = FIELD.limit(Command(-0.3, 0.0, 0.0), view, remote=True).command
    assert back.vx == pytest.approx(-CFG.remote_degraded_speed)
    side = FIELD.limit(Command(0.0, 0.3, 0.0), view, remote=True).command
    assert side.speed == 0.0  # the sector spans +-2.5 deg: exactly sideways still approaches it
    away = FIELD.limit(Command(-0.1, 0.3, 0.0), view, remote=True).command
    assert away.speed == pytest.approx(CFG.remote_degraded_speed)
    blind = np.array([-math.pi + (i + 0.5) * width for i in range(72)])
    fully = ScanView(ScanState.BLIND, np.empty((0, 2)), 72, 'x', blind, width)
    out = FIELD.limit(Command(0.3, 0.0, 0.0), fully, remote=True).command
    assert out.vx == pytest.approx(CFG.remote_degraded_speed)  # blind sensor: creep only


def close_scene(rng) -> list:
    """Return points close around the robot: clusters and wall pieces, 0.2 to 1.4 m out."""
    parts = []
    for _ in range(rng.integers(1, 5)):
        r, a = rng.uniform(0.2, 1.4), rng.uniform(-math.pi, math.pi)
        centre = np.array([r * math.cos(a), r * math.sin(a)])
        if rng.random() < 0.5:
            d, s = rng.uniform(-math.pi, math.pi), np.linspace(-0.5, 0.5, 30)
            parts.append(centre + np.column_stack((s * math.cos(d), s * math.sin(d))))
        else:
            parts.append(centre + rng.normal(0.0, 0.1, size=(rng.integers(3, 30), 2)))
    return np.vstack(parts).tolist()


def test_what_the_field_lets_through_it_lets_through_again():
    """On a limit what a command is over by is floating-point noise: it passes untouched."""
    rng = np.random.default_rng(11)
    fields = (FIELD, ProtectiveField(ROBOT, FieldConfig()))  # the person's term too
    limited = 0
    for _ in range(300):
        points = close_scene(rng)
        cmd = Command(rng.uniform(-0.3, 0.8), rng.uniform(-0.4, 0.4), rng.uniform(-1.0, 1.0))
        for field in fields:
            first = field.limit(cmd, ok(points), remote=False)
            limited += first.state is FieldState.LIMITED
            again = field.limit(first.command, ok(points), remote=False)
            assert again.state is FieldState.CLEAR and again.command == first.command
    assert limited > 50
