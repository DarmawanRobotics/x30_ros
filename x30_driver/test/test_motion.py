#!/usr/bin/env python3
"""Velocity policy and stream (episodes, takeover latch, holds, zeros) and the conversions."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from x30_driver import convert  # noqa: E402
from x30_driver import protocol as p  # noqa: E402
from x30_driver.motion import (  # noqa: E402
    AutoConfig,
    RobotView,
    ToggleGuard,
    VelocityConfig,
    VelocityPolicy,
    VelocityStream,
    ZERO,
)

DT = 0.02  # cmd_rate_hz 50
FORWARD = (0.3, 0.0, 0.0)
STOP = (0.0, 0.0, 0.0)
NO_AXES = (0, 0, 0)


def view(state=p.STATE_STEPPING, non_manual=True, gait=p.GAIT_STATE_WALK):
    """Return what fresh reports say (non_manual None: no fresh RcsData)."""
    return RobotView(state, gait, non_manual, False)


class Bench:
    """The stream stepped at 50 Hz against a robot view the test sets."""

    def __init__(self, robot, target_non_manual=True, mode='axis', **auto):
        self.stream = VelocityStream(VelocityConfig(),
                                     AutoConfig(target_non_manual=target_non_manual, **auto),
                                     mode=mode)
        self.view = robot
        self.t = 0.0
        self.steps = []  # (time, StreamStep)

    def run(self, seconds, cmd=None):
        """Step for this long; cmd: published every period, None: nothing published."""
        end = self.t + seconds - 1e-9
        while self.t < end:
            self.t = round(self.t + DT, 6)
            if cmd is not None:
                self.stream.command(*cmd, self.t)
            self.steps.append((self.t, self.stream.step(self.t, self.view)))
        return self

    def since(self, t):
        return [step for when, step in self.steps if when > t]

    def commands(self, t=-1.0):
        return [c for step in self.since(t) for c in step.commands]

    def axes(self, t=-1.0):
        return [step.axes for step in self.since(t) if step.axes is not None]

    def moving_axes(self, t=-1.0):
        return [a for a in self.axes(t) if a != NO_AXES]

    @property
    def gate(self):
        return self.stream.gate(self.t)

    @property
    def last(self):
        return self.steps[-1][1]


class TestVelocityPolicy(unittest.TestCase):
    def setUp(self):
        self.policy = VelocityPolicy(VelocityConfig(max_vx=1.0, max_vy=0.5, max_wz=1.0,
                                                    cmd_timeout_sec=0.5, stop_burst_sec=0.5))

    def test_fresh_and_moving(self):
        self.assertFalse(self.policy.moving(0.0))
        self.policy.command(0.3, 0.1, -0.2, now=10.0)
        self.assertTrue(self.policy.moving(10.5))
        self.assertFalse(self.policy.moving(10.6))
        self.assertTrue(self.policy.recently_moved(10.6, 1.0))
        self.policy.command(0.0, 0.0005, 0.0, now=11.0)  # below the epsilon: no motion
        self.assertTrue(self.policy.fresh(11.1))
        self.assertFalse(self.policy.moving(11.1))

    def test_clamp_and_non_finite_is_a_stop(self):
        self.policy.command(3.0, -2.0, 0.4, now=0.0)
        self.assertEqual(self.policy.current, (1.0, -0.5, 0.4))
        self.policy.command(0.3, float('nan'), 0.4, now=0.1)
        self.assertEqual(self.policy.current, ZERO)
        self.policy.command(float('inf'), 0.0, 0.0, now=0.2)
        self.assertEqual(self.policy.current, ZERO)

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


class TestEpisodes(unittest.TestCase):
    def test_switches_at_the_start_then_drives_at_once(self):
        bench = Bench(view(p.STATE_TORQUE_STANDING, non_manual=False))
        bench.run(DT, FORWARD)
        self.assertEqual(bench.commands(), [p.NON_MANUAL_MODE, p.STEPPING])
        bench.view = view()  # the robot switched and steps: the episode's command drives
        bench.run(DT, FORWARD)
        self.assertEqual(bench.last.axes[0], round(0.3 / 1.2 * 32767))
        self.assertEqual(bench.gate, 'driving')

    def test_mode_three_tries_on_fresh_reports_only(self):
        bench = Bench(view(non_manual=False))  # the robot never takes the mode
        bench.run(3.0, FORWARD)
        tries = [t for t, step in bench.steps if p.NON_MANUAL_MODE in step.commands]
        self.assertEqual(len(tries), 3)
        self.assertAlmostEqual(tries[1] - tries[0], 0.5)
        self.assertAlmostEqual(tries[2] - tries[1], 0.5)
        blind = Bench(view(non_manual=None))  # no fresh RcsData: never sent blindly
        blind.run(3.0, FORWARD)
        self.assertEqual(blind.commands(), [])

    def test_stepping_three_tries_through_the_guard(self):
        bench = Bench(view(p.STATE_TORQUE_STANDING))  # the robot never starts stepping
        bench.run(20.0, FORWARD)
        tries = [t for t, step in bench.steps if p.STEPPING in step.commands]
        self.assertEqual(len(tries), 3)
        self.assertGreaterEqual(tries[1] - tries[0], 5.0)  # transition_timeout_sec apart

    def test_never_mid_episode(self):
        """Stepping stopped on the handheld mid-episode is left alone; a new episode toggles."""
        bench = Bench(view(p.STATE_TORQUE_STANDING))
        bench.run(DT, FORWARD)
        self.assertEqual(bench.commands(), [p.STEPPING])
        bench.view = view()
        bench.run(1.0, FORWARD)
        bench.view = view(p.STATE_TORQUE_STANDING)
        middle = bench.t
        bench.run(5.0, FORWARD)
        self.assertEqual(bench.commands(middle), [])
        bench.run(1.2)  # quiet for cmd_timeout_sec + stop_burst_sec: the episode ends
        later = bench.t
        bench.run(DT, FORWARD)
        self.assertEqual(bench.commands(later), [p.STEPPING])

    def test_zero_twists_keep_the_episode(self):
        bench = Bench(view(p.STATE_TORQUE_STANDING))
        bench.run(DT, FORWARD)
        bench.view = view()
        bench.run(0.5, FORWARD)
        bench.run(3.0, STOP)  # zeros, not quiet
        bench.view = view(p.STATE_TORQUE_STANDING)
        resume = bench.t
        bench.run(1.0, FORWARD)
        self.assertEqual(bench.commands(resume), [])

    def test_the_gate_closing_does_not_end_the_episode(self):
        """A hold, a takeover or a fall mid-episode: open again, the stream gets no switches."""
        def hold(bench):
            bench.stream.hold(bench.t, 'stop_stepping')
            bench.view = view(p.STATE_TORQUE_STANDING)
            bench.run(1.0, STOP)
            bench.stream.release()  # stand_up: already standing

        def takeover(bench):
            bench.view = view(p.STATE_TORQUE_STANDING, non_manual=False)
            bench.run(1.0, STOP)
            bench.view = view(p.STATE_TORQUE_STANDING)  # handed back

        def fall(bench):
            bench.view = view(p.STATE_EMERGENCY)
            bench.run(1.0, FORWARD)
            bench.view = view(p.STATE_TORQUE_STANDING)  # recovered by hand

        for close in (hold, takeover, fall):
            with self.subTest(close=close.__name__):
                bench = Bench(view(p.STATE_TORQUE_STANDING))
                bench.run(DT, FORWARD)
                self.assertEqual(bench.commands(), [p.STEPPING])
                bench.view = view()
                bench.run(6.0, FORWARD)  # past the toggle guard
                close(bench)
                reopened = bench.t
                bench.run(0.1, STOP)  # armed again
                bench.run(2.0, FORWARD)
                self.assertEqual(bench.gate, 'driving')
                self.assertEqual(bench.commands(reopened), [])
                bench.run(1.2)  # quiet: the next motion starts a new episode
                later = bench.t
                bench.run(DT, FORWARD)
                self.assertEqual(bench.commands(later), [p.STEPPING])

    def test_an_episode_that_starts_closed_gets_no_switches(self):
        """The first motion after quiet came during a hold: released later, still none."""
        bench = Bench(view(p.STATE_TORQUE_STANDING, non_manual=False))
        bench.stream.hold(0.0, 'sit_down')
        bench.run(1.0, FORWARD)
        self.assertTrue(bench.stream.release())
        bench.run(DT, STOP)
        bench.run(2.0, FORWARD)
        self.assertEqual(bench.commands(), [])
        bench.run(1.2)  # quiet
        later = bench.t
        bench.run(DT, FORWARD)
        self.assertEqual(bench.commands(later), [p.NON_MANUAL_MODE, p.STEPPING])

    def test_tries_end_with_a_deadline(self):
        """A report that comes back late in the episode gets no try (zeros keep the episode)."""
        stepping = Bench(view(p.STATE_TORQUE_STANDING), auto_control_mode=False)
        stepping.run(DT, FORWARD)
        self.assertEqual(stepping.commands(), [p.STEPPING])
        start = stepping.t
        stepping.view = view(None)  # no fresh motion state report
        stepping.run(30.0, STOP)
        stepping.view = view(p.STATE_TORQUE_STANDING)  # back, still standing
        stepping.run(1.0, STOP)
        self.assertEqual(stepping.commands(start), [])
        mode = Bench(view(non_manual=False), auto_stepping=False)
        mode.run(DT, FORWARD)
        self.assertEqual(mode.commands(), [p.NON_MANUAL_MODE])
        start = mode.t
        mode.view = view(non_manual=None)  # no fresh RcsData
        mode.run(30.0, STOP)
        mode.view = view(non_manual=False)
        mode.run(1.0, STOP)
        self.assertEqual(mode.commands(start), [])
        early = Bench(view(non_manual=False), auto_stepping=False)
        early.run(DT, FORWARD)
        early.view = view(non_manual=None)
        early.run(0.6, FORWARD)
        early.view = view(non_manual=False)  # back within the deadline: the tries go on
        early.run(3.0, FORWARD)
        self.assertEqual(early.commands(), [p.NON_MANUAL_MODE] * 3)

    def test_idle_stop_stepping(self):
        bench = Bench(view(), idle_stop_stepping_sec=2.0)
        bench.run(0.5, FORWARD)
        bench.run(1.9)
        self.assertEqual(bench.commands(), [])
        bench.run(0.5)
        self.assertEqual(bench.commands(), [p.STEPPING])
        bench.run(5.0)
        self.assertEqual(bench.commands().count(p.STEPPING), 1)


class TestTakeover(unittest.TestCase):
    def test_takeover_while_cmd_vel_is_zero_latches(self):
        bench = Bench(view())
        bench.run(1.0, STOP)
        bench.view = view(non_manual=False)  # manual on the handheld, no motion asked
        bench.run(DT, STOP)
        self.assertEqual(bench.gate, 'takeover (manual mode)')
        self.assertIn('took over', bench.stream.latch_reason)
        start = bench.t
        bench.run(2.0, FORWARD)
        self.assertEqual(bench.commands(start), [])
        self.assertEqual(bench.moving_axes(start), [])

    def test_latch_survives_zeros_silence_and_motion(self):
        bench = Bench(view())
        bench.run(1.0, FORWARD)
        bench.view = view(non_manual=False)
        start = bench.t
        bench.run(5.0, STOP)
        bench.run(5.0)
        bench.run(2.0, FORWARD)
        self.assertEqual(bench.gate, 'takeover (manual mode)')
        self.assertEqual(bench.commands(start), [])
        self.assertEqual(bench.moving_axes(start), [])
        self.assertIn('takeover', bench.last.ignored)

    def test_latch_ends_on_the_target_mode_then_needs_zero_or_quiet(self):
        bench = Bench(view())
        bench.run(1.0, FORWARD)
        bench.view = view(non_manual=False)
        bench.run(1.0, FORWARD)
        bench.view = view()  # handed back on the handheld; cmd_vel still streams
        back = bench.t
        bench.run(1.0, FORWARD)
        self.assertEqual(bench.gate, 'waiting for zero or quiet cmd_vel')
        self.assertEqual(bench.moving_axes(back), [])
        bench.run(DT, STOP)
        bench.run(DT, FORWARD)
        self.assertEqual(bench.gate, 'driving')
        self.assertEqual(bench.last.axes[0], round(0.3 / 1.2 * 32767))
        quiet = Bench(view())
        quiet.run(1.0, FORWARD)
        quiet.view = view(non_manual=False)
        quiet.run(1.0)
        quiet.view = view()
        quiet.run(DT)  # cmd_vel quiet already
        self.assertEqual(quiet.gate, 'idle')

    def test_mode_service_latch_and_handback(self):
        """manual_mode latches even before the report; non_manual_mode ends it on the report."""
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        bench.stream.latch(bench.t, 'manual_mode service')
        bench.run(2.0, FORWARD)  # the robot did not take it: still non-manual
        self.assertEqual(bench.gate, 'takeover (manual mode)')
        self.assertEqual(bench.stream.latch_reason, 'manual_mode service')
        bench.stream.request_handback()
        bench.run(DT, FORWARD)
        self.assertEqual(bench.gate, 'waiting for zero or quiet cmd_vel')

    def test_remote_prefix_targets_manual_mode(self):
        bench = Bench(view(non_manual=True), target_non_manual=False)
        bench.run(DT, FORWARD)
        self.assertEqual(bench.commands(), [p.MANUAL_MODE])
        bench.view = view(non_manual=False)
        bench.run(0.5, FORWARD)
        bench.view = view(non_manual=True)
        bench.run(DT, FORWARD)
        self.assertEqual(bench.gate, 'takeover (non-manual mode)')


class TestHolds(unittest.TestCase):
    def test_holds_need_a_release_then_zero_or_quiet(self):
        for service in ('sit_down', 'emergency_stop', 'stop_stepping'):
            with self.subTest(service=service):
                bench = Bench(view())
                bench.run(0.5, FORWARD)
                bench.stream.hold(bench.t, service)
                bench.run(5.0, STOP)
                bench.run(5.0)
                bench.run(1.0, FORWARD)
                self.assertEqual(bench.gate, 'hold:' + service)
                self.assertTrue(bench.stream.release())
                bench.run(1.0, FORWARD)
                self.assertEqual(bench.gate, 'waiting for zero or quiet cmd_vel')
                bench.run(DT, STOP)
                bench.run(DT, FORWARD)
                self.assertEqual(bench.gate, 'driving')
                self.assertFalse(bench.stream.release())

    def test_zero_burst_after_a_hold(self):
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        bench.stream.hold(bench.t, 'stop_stepping')
        start = bench.t
        bench.run(2.0, FORWARD)
        self.assertEqual(bench.axes(start), [NO_AXES] * 25)  # stop_burst_sec at 50 Hz

    def test_emergency_report_disarms_until_it_clears(self):
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        bench.view = view(p.STATE_EMERGENCY)
        bench.run(1.0, FORWARD)
        self.assertEqual(bench.gate, 'disarmed:emergency stop or fall')
        bench.view = view(p.STATE_SITTING)
        bench.run(DT, FORWARD)
        self.assertEqual(bench.gate, 'waiting for zero or quiet cmd_vel')
        bench.run(DT, STOP)
        self.assertEqual(bench.gate, 'idle')


class TestZeros(unittest.TestCase):
    def test_nan_in_a_field_is_a_stop(self):
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        start = bench.t
        bench.run(1.0, (0.3, float('nan'), 0.0))
        self.assertFalse(bench.stream.policy.moving(bench.t))
        self.assertEqual(bench.moving_axes(start), [])

    def test_zero_twists_send_only_the_burst(self):
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        start = bench.t
        bench.run(4.0, STOP)
        self.assertEqual(bench.axes(start), [NO_AXES] * 25)

    def test_stale_command_then_the_burst(self):
        bench = Bench(view())
        bench.run(0.5, FORWARD)
        start = bench.t
        bench.run(2.0)  # nothing published: fresh for cmd_timeout_sec, then zeros, then silence
        axes = bench.axes(start)
        self.assertEqual(len(bench.moving_axes(start)), 25)
        self.assertEqual(axes[25:], [NO_AXES] * 25)

    def test_no_zeros_without_motion_before(self):
        self.assertEqual(Bench(view()).run(3.0, STOP).axes(), [])

    def test_topic_mode(self):
        bench = Bench(view(p.STATE_TORQUE_STANDING, non_manual=False), mode='topic')
        bench.run(DT, FORWARD)
        self.assertEqual((bench.last.twist, bench.last.axes), (FORWARD, None))
        self.assertEqual(bench.commands(), [p.NON_MANUAL_MODE, p.STEPPING])
        start = bench.t
        bench.run(4.0, STOP)
        self.assertEqual([s.twist for s in bench.since(start) if s.twist is not None],
                         [ZERO] * 25)

    def test_gating(self):
        bench = Bench(view(None))
        bench.run(DT, FORWARD)
        self.assertEqual((bench.last.axes, bench.last.ignored),
                         (None, 'no fresh motion state report'))
        bench.view = view(p.STATE_TORQUE_STANDING)
        bench.run(DT, FORWARD)
        self.assertIsNone(bench.last.axes)  # torque standing: axes would change the pose
        bench.view = view(gait=p.GAIT_STATE_SILENT)
        bench.run(DT, FORWARD)
        self.assertIsNone(bench.last.axes)
        self.assertIn('silent', bench.last.ignored)


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
