#!/usr/bin/env python3
"""Velocity policy, automatic mode / stepping switches and the report conversions."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from x30_driver import convert  # noqa: E402
from x30_driver import protocol as p  # noqa: E402
from x30_driver.motion import (  # noqa: E402
    AutoConfig,
    AutoSwitch,
    VelocityConfig,
    VelocityPolicy,
    ZERO,
)


class TestVelocityPolicy(unittest.TestCase):
    def setUp(self):
        self.policy = VelocityPolicy(VelocityConfig(max_vx=1.0, max_vy=0.5, max_wz=1.0,
                                                    cmd_timeout_sec=0.5, stop_burst_sec=0.5))

    def test_silent_before_the_first_command(self):
        self.assertIsNone(self.policy.tick(0.0))
        self.assertFalse(self.policy.moving(0.0))
        self.assertEqual(self.policy.sent_command(0.0), ZERO)

    def test_watchdog(self):
        """The command while fresh, then zeros for stop_burst_sec, then nothing."""
        self.policy.command(0.3, 0.1, -0.2, now=10.0)
        self.assertEqual(self.policy.tick(10.4), (0.3, 0.1, -0.2))
        self.assertTrue(self.policy.moving(10.4))
        self.assertEqual(self.policy.tick(10.6), ZERO)
        self.assertFalse(self.policy.moving(10.6))
        self.assertEqual(self.policy.sent_command(10.6), ZERO)
        self.assertEqual(self.policy.tick(10.9), ZERO)
        self.assertIsNone(self.policy.tick(11.1))

    def test_clamp_and_non_finite(self):
        self.policy.command(3.0, -2.0, float('nan'), now=0.0)
        self.assertEqual(self.policy.current, (1.0, -0.5, 0.0))
        self.policy.command(float('inf'), 0.0, -9.0, now=0.0)
        self.assertEqual(self.policy.current, (0.0, 0.0, -1.0))

    def test_stop_streams_zeros(self):
        self.policy.command(0.5, 0.0, 0.0, now=0.0)
        self.policy.stop(now=0.1)
        self.assertEqual(self.policy.tick(0.2), ZERO)
        self.assertFalse(self.policy.moving(0.2))
        self.assertTrue(self.policy.recently_moved(0.2, 1.0))
        self.assertFalse(self.policy.recently_moved(1.5, 1.0))

    def test_zero_command_is_not_motion(self):
        self.policy.command(0.0, 0.0005, 0.0, now=0.0)
        self.assertTrue(self.policy.fresh(0.1))
        self.assertFalse(self.policy.moving(0.1))
        self.assertFalse(self.policy.recently_moved(0.1, 1.0))

    def test_axes_follow_the_gait(self):
        """Scaled by the current gait's Appendix B speeds; crawl halves walk forward."""
        cmd = (0.3, 0.0, 0.0)
        self.assertEqual(self.policy.axes(cmd, p.GAIT_STATE_WALK)[0], round(0.3 / 1.2 * 32767))
        self.assertEqual(self.policy.axes(cmd, p.GAIT_STATE_OFF_ROAD)[0], 32767)
        self.assertEqual(self.policy.axes(cmd, p.GAIT_STATE_WALK, crawl=True)[0],
                         round(0.3 / 0.6 * 32767))
        self.assertEqual(self.policy.axes(cmd, None)[0], round(0.3 / 1.2 * 32767))

    def test_axes_signs_and_inverts(self):
        """REP-103 left / ccw are the robot's negative axes; invert_* flip them back."""
        self.assertEqual(self.policy.axes((0.0, 0.1, 0.2), p.GAIT_STATE_L_WALK),
                         (0, -round(0.1 / 0.5 * 32767), -round(0.2 / 1.2 * 32767)))
        inverted = VelocityPolicy(VelocityConfig(invert_vy=True, invert_wz=True))
        self.assertEqual(inverted.axes((0.0, 0.1, 0.2), p.GAIT_STATE_L_WALK),
                         (0, round(0.1 / 0.5 * 32767), round(0.2 / 1.2 * 32767)))

    def test_axes_dead_zone_parameter(self):
        cmd = (0.001, 0.0, 0.0)
        self.assertEqual(self.policy.axes(cmd, p.GAIT_STATE_WALK), (656, 0, 0))
        raw = VelocityPolicy(VelocityConfig(dead_zone=0))
        self.assertEqual(raw.axes(cmd, p.GAIT_STATE_WALK), (27, 0, 0))


class TestAutoSwitch(unittest.TestCase):
    def setUp(self):
        self.auto = AutoSwitch(AutoConfig(mode_refresh_sec=2.0, stepping_retry_sec=2.0))

    def test_idle_sends_nothing(self):
        self.assertEqual(self.auto.decide(0.0, False, p.STATE_TORQUE_STANDING, False), [])

    def test_motion_from_torque_standing(self):
        """Non-manual mode and the stepping toggle, each rate-limited."""
        self.assertEqual(self.auto.decide(0.0, True, p.STATE_TORQUE_STANDING, False),
                         [p.NON_MANUAL_MODE, p.STEPPING])
        self.assertEqual(self.auto.decide(1.0, True, p.STATE_TORQUE_STANDING, False), [])
        self.assertEqual(self.auto.decide(2.0, True, p.STATE_TORQUE_STANDING, False),
                         [p.NON_MANUAL_MODE, p.STEPPING])

    def test_no_toggle_outside_torque_standing(self):
        """The stepping command is a toggle: never sent while stepping or sitting."""
        for state in (p.STATE_STEPPING, p.STATE_SITTING, p.STATE_INITIAL_STANDING,
                      p.STATE_EMERGENCY, None):
            auto = AutoSwitch(AutoConfig())
            self.assertNotIn(p.STEPPING, auto.decide(0.0, True, state, True), state)

    def test_mode_only_while_not_reported(self):
        self.assertEqual(self.auto.decide(0.0, True, p.STATE_STEPPING, True), [])
        self.assertEqual(self.auto.decide(0.0, True, p.STATE_STEPPING, None),
                         [p.NON_MANUAL_MODE])

    def test_switches_can_be_disabled(self):
        auto = AutoSwitch(AutoConfig(auto_non_manual_mode=False, auto_stepping=False))
        self.assertEqual(auto.decide(0.0, True, p.STATE_TORQUE_STANDING, False), [])

    def test_idle_stop_stepping(self):
        """Off by default; with a delay, stepping stops once the robot stood idle that long."""
        self.auto.decide(0.0, True, p.STATE_STEPPING, True)
        self.assertEqual(self.auto.decide(60.0, False, p.STATE_STEPPING, True), [])
        auto = AutoSwitch(AutoConfig(idle_stop_stepping_sec=5.0))
        self.assertEqual(auto.decide(0.0, False, p.STATE_STEPPING, True), [])  # never moved
        auto.decide(1.0, True, p.STATE_STEPPING, True)
        self.assertEqual(auto.decide(4.0, False, p.STATE_STEPPING, True), [])
        self.assertEqual(auto.decide(6.0, False, p.STATE_STEPPING, True), [p.STEPPING])
        self.assertEqual(auto.decide(7.0, False, p.STATE_TORQUE_STANDING, True), [])
        # stepping started again by hand (start_stepping) without motion is left alone
        self.assertEqual(auto.decide(20.0, False, p.STATE_STEPPING, True), [])


class TestConvert(unittest.TestCase):
    def test_quaternion(self):
        q = convert.quaternion_from_rpy(0.0, 0.0, math.pi / 2)
        for got, want in zip(q, (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))):
            self.assertAlmostEqual(got, want)
        q = convert.quaternion_from_rpy(0.3, -0.2, 1.0)
        self.assertAlmostEqual(sum(v * v for v in q), 1.0)

    def test_attitude_in_radians(self):
        sensor = p.SensorData(0, (90.0, -45.0, 180.0), (0.0,) * 3, (0.0,) * 3,
                              (0.0,) * 12, (0.0,) * 12, (0.0,) * 12)
        for got, want in zip(convert.attitude(sensor), (math.pi / 2, -math.pi / 4, math.pi)):
            self.assertAlmostEqual(got, want)
        self.assertEqual(convert.attitude(None), (0.0, 0.0, 0.0))

    def test_odometry_orientation(self):
        """IMU roll and pitch, the leg odometry's yaw."""
        motion = p.MotionState(4, 0, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0))
        self.assertEqual(convert.odometry_orientation(motion, None),
                         convert.quaternion_from_rpy(0.0, 0.0, 0.5))
        sensor = p.SensorData(0, (10.0, 5.0, 99.0), (0.0,) * 3, (0.0,) * 3,
                              (0.0,) * 12, (0.0,) * 12, (0.0,) * 12)
        self.assertEqual(convert.odometry_orientation(motion, sensor),
                         convert.quaternion_from_rpy(math.radians(10.0), math.radians(5.0), 0.5))

    def test_battery(self):
        self.assertEqual(convert.battery_voltage(54), 54.0)
        self.assertAlmostEqual(convert.battery_voltage(5430), 54.3)
        self.assertAlmostEqual(convert.battery_voltage(54300), 54.3)
        self.assertAlmostEqual(convert.battery_voltage(543, scale=0.1), 54.3)
        self.assertEqual(convert.battery_fraction(87), 0.87)
        self.assertEqual(convert.battery_fraction(250), 1.0)
        self.assertEqual(convert.max_finite([1.0, float('nan'), 3.0]), 3.0)
        self.assertTrue(math.isnan(convert.max_finite([])))
        self.assertEqual(convert.finite_or(float('inf'), 2.0), 2.0)
        self.assertEqual(convert.finite_or(1.5, 2.0), 1.5)


if __name__ == '__main__':
    unittest.main()
