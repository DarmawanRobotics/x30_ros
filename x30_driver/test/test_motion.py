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
    RobotView,
    ToggleGuard,
    VelocityConfig,
    VelocityPolicy,
    VelocityStream,
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
        self.assertIsNone(self.policy.axes(cmd, None))
        self.assertIsNone(self.policy.axes(cmd, p.GAIT_STATE_SILENT))  # not in Appendix B
        given = VelocityPolicy(VelocityConfig(gait_limits=p.limits_table(
            p.parse_gait_limits({'silent': [0.6, 0.4, 0.2, 0.6]}))))
        self.assertEqual(given.axes(cmd, p.GAIT_STATE_SILENT)[0], round(0.3 / 0.6 * 32767))

    def test_axes_signs_and_inverts(self):
        """REP-103 left / ccw are the robot's negative axes; invert_* flip them back."""
        self.assertEqual(self.policy.axes((0.0, 0.1, 0.2), p.GAIT_STATE_L_WALK),
                         (0, -round(0.1 / 0.5 * 32767), -round(0.2 / 1.2 * 32767)))
        inverted = VelocityPolicy(VelocityConfig(invert_vy=True, invert_wz=True))
        self.assertEqual(inverted.axes((0.0, 0.1, 0.2), p.GAIT_STATE_L_WALK),
                         (0, round(0.1 / 0.5 * 32767), round(0.2 / 1.2 * 32767)))

    def test_axes_dead_zone_parameter(self):
        cmd = (0.002, 0.0, 0.0)
        self.assertEqual(self.policy.axes(cmd, p.GAIT_STATE_WALK), (656, 0, 0))
        raw = VelocityPolicy(VelocityConfig(dead_zone=0))
        self.assertEqual(raw.axes(cmd, p.GAIT_STATE_WALK), (55, 0, 0))
        # noise: no creep
        self.assertEqual(self.policy.axes((5e-4, -5e-4, 9e-4), p.GAIT_STATE_WALK), (0, 0, 0))


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
        auto = AutoSwitch(AutoConfig(auto_control_mode=False, auto_stepping=False))
        self.assertEqual(auto.decide(0.0, True, p.STATE_TORQUE_STANDING, False), [])

    def test_operator_takeover_latches(self):
        """Non-manual seen, then manual while moving: no mode command until cmd_vel rests."""
        auto = self.auto
        self.assertEqual(auto.decide(0.0, True, p.STATE_STEPPING, False), [p.NON_MANUAL_MODE])
        self.assertEqual(auto.decide(0.1, True, p.STATE_STEPPING, True), [])
        self.assertEqual(auto.decide(0.2, True, p.STATE_STEPPING, False), [])  # handheld
        self.assertFalse(auto.armed)
        self.assertIn('took over', auto.disarm_reason)
        for t in (2.5, 5.0, 10.0):  # cmd_vel keeps coming: never reverted
            self.assertEqual(auto.decide(t, True, p.STATE_STEPPING, False), [])
        self.assertEqual(auto.decide(10.3, False, p.STATE_STEPPING, False), [])  # resting
        self.assertFalse(auto.armed)
        self.assertEqual(auto.decide(10.6, False, p.STATE_STEPPING, False), [])
        self.assertTrue(auto.armed)  # quiet for rearm_quiet_sec
        self.assertEqual(auto.decide(11.0, True, p.STATE_STEPPING, False), [p.NON_MANUAL_MODE])

    def test_manual_at_start_is_not_a_takeover(self):
        """Before the robot was seen in the target mode, manual is just where it starts."""
        self.assertEqual(self.auto.decide(0.0, True, p.STATE_STEPPING, False),
                         [p.NON_MANUAL_MODE])
        self.assertEqual(self.auto.decide(0.5, True, p.STATE_STEPPING, False), [])
        self.assertTrue(self.auto.armed)
        # leaving the mode while idle does not latch either
        self.auto.decide(1.0, False, p.STATE_STEPPING, True)
        self.auto.decide(1.5, False, p.STATE_STEPPING, False)
        self.assertTrue(self.auto.armed)

    def test_disarm_holds_until_quiet(self):
        """A service stop (disarm) is not undone by the next cmd_vel."""
        auto = self.auto
        auto.disarm(0.0, 'stop_stepping')
        for t in (0.1, 0.4, 3.0):
            self.assertEqual(auto.decide(t, True, p.STATE_TORQUE_STANDING, True), [])
        self.assertEqual(auto.decide(3.3, False, p.STATE_TORQUE_STANDING, True), [])
        self.assertEqual(auto.decide(3.6, False, p.STATE_TORQUE_STANDING, True), [])
        self.assertTrue(auto.armed)
        self.assertEqual(auto.decide(3.7, True, p.STATE_TORQUE_STANDING, True), [p.STEPPING])
        quiet = AutoSwitch(AutoConfig())
        quiet.disarm(0.0, 'sit_down')  # no cmd_vel at all: armed after the quiet time
        self.assertEqual(quiet.decide(0.2, False, p.STATE_SITTING, None), [])
        self.assertFalse(quiet.armed)
        quiet.decide(0.6, False, p.STATE_SITTING, None)
        self.assertTrue(quiet.armed)
        quiet.disarm(1.0, 'manual_mode')
        quiet.arm()
        self.assertEqual((quiet.armed, quiet.disarm_reason), (True, ''))

    def test_emergency_disarms(self):
        auto = self.auto
        for t in (0.0, 1.0, 5.0):
            self.assertEqual(auto.decide(t, False, p.STATE_EMERGENCY, True), [])
            self.assertFalse(auto.armed)
        self.assertEqual(auto.decide(5.2, False, p.STATE_SITTING, True), [])
        self.assertFalse(auto.armed)  # quiet time counts from the last emergency report
        auto.decide(5.6, False, p.STATE_SITTING, True)
        self.assertTrue(auto.armed)

    def test_target_follows_the_prefix(self):
        """With 0x21 (acting as the handheld) the robot obeys in manual mode."""
        auto = AutoSwitch(AutoConfig(target_non_manual=False))
        self.assertEqual(auto.mode_command, p.MANUAL_MODE)
        self.assertEqual(auto.decide(0.0, True, p.STATE_STEPPING, True), [p.MANUAL_MODE])
        self.assertEqual(auto.decide(0.1, True, p.STATE_STEPPING, False), [])
        self.assertEqual(auto.decide(0.2, True, p.STATE_STEPPING, True), [])  # takeover
        self.assertIn('non-manual', auto.disarm_reason)

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


class TestToggleGuard(unittest.TestCase):
    def test_one_toggle_until_the_state_shows_it(self):
        guard = ToggleGuard(timeout_sec=5.0)
        self.assertTrue(guard.request(0.0, p.STEPPING, p.STATE_TORQUE_STANDING))
        self.assertEqual(guard.pending(0.1, p.STATE_TORQUE_STANDING), p.STEPPING)
        self.assertFalse(guard.request(0.1, p.STEPPING, p.STATE_TORQUE_STANDING))
        self.assertFalse(guard.request(0.2, p.STAND_SIT, None))  # unknown state: wait
        self.assertTrue(guard.request(0.3, p.STEPPING, p.STATE_STEPPING))  # shown: next one
        self.assertFalse(guard.request(0.4, p.STAND_SIT, p.STATE_STEPPING))
        self.assertTrue(guard.request(5.4, p.STAND_SIT, p.STATE_STEPPING))  # timed out
        guard.cancel()
        self.assertIsNone(guard.pending(5.5, p.STATE_STEPPING))


class TestVelocityStream(unittest.TestCase):
    def setUp(self):
        self.stream = VelocityStream(VelocityConfig(), AutoConfig())
        self.stepping = RobotView(p.STATE_STEPPING, p.GAIT_STATE_WALK, True)

    def test_axes_while_stepping(self):
        self.stream.command(0.3, 0.0, 0.0, 0.0)
        step = self.stream.step(0.02, self.stepping)
        self.assertEqual((step.commands, step.axes[0], step.ignored),
                         ([], round(0.3 / 1.2 * 32767), ''))

    def test_no_axes_without_a_fresh_motion_state(self):
        self.stream.command(0.3, 0.0, 0.0, 0.0)
        step = self.stream.step(0.02, RobotView(None, None, True))
        self.assertIsNone(step.axes)
        self.assertEqual(step.ignored, 'no fresh motion state report')
        step = self.stream.step(0.04, RobotView(p.STATE_TORQUE_STANDING, 0, True))
        self.assertIsNone(step.axes)  # torque standing: axes would change the pose

    def test_unknown_gait_streams_zeros(self):
        self.stream.command(0.3, 0.0, 0.0, 0.0)
        step = self.stream.step(0.02, RobotView(p.STATE_STEPPING, p.GAIT_STATE_SILENT, True))
        self.assertEqual(step.axes, (0, 0, 0))
        self.assertIn('silent', step.ignored)

    def test_disarmed_streams_zeros(self):
        self.stream.command(0.3, 0.0, 0.0, 0.0)
        self.stream.disarm(0.01, 'stop_stepping')
        self.stream.command(0.3, 0.0, 0.0, 0.02)
        step = self.stream.step(0.03, self.stepping)
        self.assertEqual((step.commands, step.axes), ([], (0, 0, 0)))
        self.assertIn('stop_stepping', step.ignored)
        self.assertEqual(self.stream.sent_command(0.03), ZERO)
        self.assertIsNone(self.stream.step(2.0, self.stepping).axes)  # quiet: silence
        self.stream.command(0.3, 0.0, 0.0, 2.1)
        self.assertEqual(self.stream.step(2.12, self.stepping).axes[0], 8192)  # armed again

    def test_topic_mode(self):
        stream = VelocityStream(VelocityConfig(), AutoConfig(), mode='topic')
        stream.command(0.3, 0.1, -0.2, 0.0)
        step = stream.step(0.02, RobotView(p.STATE_TORQUE_STANDING, 0, False))
        self.assertEqual((step.twist, step.axes), ((0.3, 0.1, -0.2), None))
        self.assertEqual(step.commands, [p.NON_MANUAL_MODE, p.STEPPING])


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
