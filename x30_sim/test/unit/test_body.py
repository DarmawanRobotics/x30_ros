"""Body: acceleration limits, measured and clamped dt, diff kinematics, collision latch."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_sim.core.body import Body, BodyLimits, Twist, integrate
from x30_sim.core.latency import CommandQueue
from x30_sim.core.world import World
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile

LIMITS = BodyLimits(accel=1.0, decel=2.0, yaw_accel=2.0, max_dt=0.1)
CORRIDOR = World.from_dict({'walls': [[[2.0, -5.0], [2.0, 5.0]]]})


def run(body: Body, command: Twist, seconds: float, dt: float = 0.01, t0: float = 0.0) -> float:
    """Step the body with a fixed command; return the final time."""
    t = t0
    body.step(t, command)
    for _ in range(round(seconds / dt)):
        t += dt
        body.step(t, command)
    return t


def test_follows_commands_with_acceleration_limits():
    """1 m/s forward takes 1 s at 1 m/s^2; braking uses the (faster) deceleration."""
    body = Body(Pose2D(), LIMITS)
    t = run(body, Twist(1.0), 1.0)
    assert body.twist.vx == pytest.approx(1.0) and body.pose.x == pytest.approx(0.5, abs=0.01)
    run(body, Twist(), 0.5, t0=t)
    assert body.twist.vx == pytest.approx(0.0, abs=1e-9)
    assert body.pose.x == pytest.approx(0.75, abs=0.01)  # v^2 / 2 decel = 0.25 m


def test_sideways_while_facing_y_moves_to_minus_x():
    """Body-frame commands: +vy (left) while facing +y moves to -x."""
    body = Body(Pose2D(yaw=math.pi / 2), BodyLimits(accel=100.0, decel=100.0))
    run(body, Twist(0.0, 0.5), 2.0)
    assert body.pose.x == pytest.approx(-1.0, abs=0.02) and abs(body.pose.y) < 1e-9


def test_diff_drive_ignores_vy():
    """A diff-drive body never moves sideways."""
    body = Body(Pose2D(), BodyLimits(omni=False))
    run(body, Twist(0.0, 0.5, 0.0), 1.0)
    assert body.twist.vy == 0.0 and body.pose.y == 0.0
    profile = RobotProfile(kinematics='diff')
    assert not BodyLimits.from_robot(profile, 0.1).omni


def test_measured_dt_not_ticks():
    """#14: the body integrates the time that passed, not one nominal tick per call."""
    fast, slow = Body(Pose2D(), BodyLimits(accel=100.0)), Body(Pose2D(), BodyLimits(accel=100.0))
    run(fast, Twist(1.0), 1.0, dt=0.01)  # 100 steps
    run(slow, Twist(1.0), 1.0, dt=0.05)  # 20 steps, as under load
    assert slow.pose.x == pytest.approx(fast.pose.x, abs=0.02)
    assert fast.pose.x == pytest.approx(1.0, abs=0.02)


def test_stall_is_clamped_to_max_dt():
    """A 5 s stall moves the body by at most max_dt of motion."""
    body = Body(Pose2D(), BodyLimits(accel=100.0, max_dt=0.1))
    t = run(body, Twist(1.0), 1.0)
    x = body.pose.x
    assert body.step(t + 5.0, Twist(1.0)) == pytest.approx(0.1)
    assert body.pose.x - x == pytest.approx(0.1, abs=1e-6)
    assert body.step(t + 4.0, Twist(1.0)) == 0.0  # time went back: no step


def test_collision_stops_and_latches():
    """Driving into a wall stops the body at contact and latches the collision."""
    box = ((-0.3, -0.2), (0.3, 0.2))
    body = Body(Pose2D(), BodyLimits(accel=100.0, decel=100.0), box, CORRIDOR)
    t = run(body, Twist(1.0), 3.0)
    assert body.collided and body.twist == Twist()
    assert 1.6 < body.pose.x < 1.7 + 1e-9  # front edge stopped just before x = 2
    run(body, Twist(-0.5), 1.0, t0=t)  # backing away is allowed, the flag stays
    assert body.collided and body.pose.x < 1.3


def test_obstacle_dropped_on_the_robot_is_a_contact():
    """A circle placed onto the footprint latches the collision; the body can leave it."""
    box = ((-0.3, -0.2), (0.3, 0.2))
    body = Body(Pose2D(), BodyLimits(accel=100.0), box, World())
    person = np.array([[0.0, 0.0, 0.25, 1.7]])
    body.step(0.0, Twist(-1.0), person)
    body.step(0.05, Twist(-1.0), person)
    assert body.collided and body.pose.x < 0.0


def test_stale_command_brakes_to_zero():
    """The command queue returns zero after the timeout: the body stops like the driver."""
    queue = CommandQueue(0.0, 0.5, Twist())
    body = Body(Pose2D(), LIMITS)
    queue.push(0.0, Twist(0.5))
    t = 0.0
    body.step(t, queue.active(t))
    for _ in range(200):
        t += 0.01
        body.step(t, queue.active(t))
    assert body.twist.vx == 0.0
    assert body.pose.x == pytest.approx(0.125 + 0.0625, abs=0.01)  # 0.5 s up, then braking


def test_integrate_is_an_exact_arc():
    """A quarter circle of radius 1 ends at (1, 1) facing +y."""
    pose = integrate(Pose2D(), Twist(math.pi / 2, 0.0, math.pi / 2), 1.0)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 1.0, math.pi / 2))
