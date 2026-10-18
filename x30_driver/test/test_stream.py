#!/usr/bin/env python3
"""The velocity stream in a closed loop with the simulated robot, in simulated time."""

import collections
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, HERE)  # nose -P
from fake_robot import AXES, FakeX30  # noqa: E402
from x30_driver import protocol as p  # noqa: E402
from x30_driver.motion import (  # noqa: E402
    AutoConfig,
    RobotView,
    VelocityConfig,
    VelocityStream,
)

DT = 0.02  # cmd_rate_hz 50
FORWARD = (0.3, 0.0, 0.0)
STOP = (0.0, 0.0, 0.0)


class Loop:
    """The driver's send_velocity against a FakeX30; reports arrive `lag` periods late."""

    def __init__(self, robot, prefix=p.PREFIX_AUTONOMOUS, lag=0, velocity=None, **auto):
        self.robot = robot
        self.prefix = prefix
        self.stream = VelocityStream(velocity or VelocityConfig(),
                                     AutoConfig(target_non_manual=prefix == p.PREFIX_AUTONOMOUS,
                                                **auto))
        self.t = 0.0
        self.sent = []      # (time, command base) of every mode / toggle command
        self.axes_log = []  # (time, axis values) of every period with axis commands
        self.step = None
        self._views = collections.deque(maxlen=lag + 1)

    def send(self, base, value=0):
        self.sent.append((self.t, base))
        self.robot.handle(p.simple(p.code(base, self.prefix), value))

    def toggle(self, base):
        """send_toggle of the services: through the stream's guard."""
        if not self.stream.toggles.request(self.t, base, self.view().basic_state):
            return False
        self.send(base)
        return True

    def view(self):
        return self._views[0] if self._views else RobotView()

    def sent_since(self, t, base):
        return [s for s in self.sent if s[0] >= t and s[1] == base]

    def axis_datagrams(self, t):
        """Return the axis values the robot received since t, one tuple per period."""
        return [s for s in self.axes_log if s[0] >= t]

    def stall(self, seconds):
        """Stall the timer (load): no period for this long."""
        self.t = round(self.t + seconds, 6)

    def run(self, seconds, cmd=None):
        """Run the loop; cmd: (vx, vy, wz) published every period, None: nothing published."""
        end = self.t + seconds - 1e-9
        while self.t < end:
            self.t = round(self.t + DT, 6)
            r = self.robot
            r.advance(self.t)
            self._views.append(RobotView(r.basic_state if r.send_motion else None, r.gait_state,
                                         r.non_manual, r.height == -1))
            if cmd is not None:
                self.stream.command(*cmd, self.t)
            self.step = self.stream.step(self.t, self.view())
            for base in self.step.commands:
                self.send(base)  # the stream already passed its toggles through the guard
            if self.step.axes is not None:
                self.axes_log.append((self.t, self.step.axes))
                for axis, value in zip(AXES, self.step.axes):
                    r.handle(p.axis(p.code(axis, self.prefix), value))
        return self.robot


class TestClosedLoop(unittest.TestCase):
    def test_drives_in_non_manual_mode(self):
        """0x31: non-manual mode and stepping at the episode start, then the commanded speed."""
        run = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING))
        robot = run.run(1.0, FORWARD)
        self.assertTrue(robot.non_manual)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)
        self.assertEqual(run.stream.gate(run.t), 'driving')

    def test_handheld_takeover_is_a_latch(self):
        """Manual on the handheld: no switch back through zeros, silence and new motion."""
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING))
        run.run(1.0, FORWARD)
        run.robot.set_mode(False)  # the operator switches on the handheld
        takeover = run.t
        robot = run.run(3.0, FORWARD)
        robot = run.run(5.0, STOP)
        robot = run.run(5.0)
        robot = run.run(2.0, FORWARD)
        self.assertFalse(robot.non_manual)
        self.assertEqual(run.sent_since(takeover, p.NON_MANUAL_MODE), [])
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        self.assertEqual(run.stream.gate(run.t), 'takeover (manual mode)')
        robot.set_mode(True)  # handed back on the handheld; cmd_vel still streams
        robot = run.run(1.0, FORWARD)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        self.assertEqual(run.stream.gate(run.t), 'waiting for zero or quiet cmd_vel')
        run.run(DT, STOP)
        robot = run.run(1.0, FORWARD)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)

    def test_takeover_while_idle_latches(self):
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        run.run(1.0, STOP)
        run.robot.set_mode(False)
        idle = run.t
        robot = run.run(3.0, FORWARD)
        self.assertFalse(robot.non_manual)
        self.assertEqual(run.sent_since(idle, p.NON_MANUAL_MODE), [])
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))

    def test_remote_prefix_drives_in_manual_mode(self):
        """0x21 (acting as the handheld): the robot obeys in manual mode, so that is the target."""
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True),
                   prefix=p.PREFIX_REMOTE)
        robot = run.run(1.0, FORWARD)
        self.assertFalse(robot.non_manual)
        self.assertEqual(run.sent[0][1], p.MANUAL_MODE)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)
        stuck = Loop(FakeX30(basic_state=p.STATE_STEPPING), auto_control_mode=False)
        self.assertEqual(stuck.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertIn('manual mode', stuck.step.ignored)

    def test_stop_stepping_holds_until_start_stepping(self):
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        run.run(1.0, FORWARD)
        run.stream.hold(run.t, 'stop_stepping')  # what the service does
        self.assertTrue(run.toggle(p.STEPPING))
        stop = run.t
        robot = run.run(3.0, FORWARD)
        robot = run.run(2.0)
        robot = run.run(1.0, FORWARD)  # a new episode: still held
        self.assertEqual(robot.basic_state, p.STATE_TORQUE_STANDING)
        self.assertEqual(run.sent_since(stop + DT, p.STEPPING), [])
        run.stream.release()  # start_stepping
        self.assertTrue(run.toggle(p.STEPPING))
        robot = run.run(1.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))  # the stream kept running
        run.run(DT, STOP)
        self.assertAlmostEqual(run.run(1.0, FORWARD).velocity[0], 0.3, places=3)

    def test_emergency_stop_holds_back(self):
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        run.run(0.5, FORWARD)
        run.robot.handle(p.simple(p.code(p.SOFT_ESTOP)))
        estop = run.t
        run.run(3.0, FORWARD)
        self.assertEqual(run.sent_since(estop, p.STEPPING), [])
        self.assertEqual(run.stream.gate(run.t), 'disarmed:emergency stop or fall')

    def test_stepping_stopped_mid_episode_stays(self):
        """The handheld stops stepping while cmd_vel streams: no toggle until a new episode."""
        run = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True))
        run.run(1.0, FORWARD)
        run.robot.handle(p.simple(p.code(p.STEPPING, p.PREFIX_REMOTE)))  # the handheld
        middle = run.t
        robot = run.run(4.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_TORQUE_STANDING)
        self.assertEqual(run.sent_since(middle, p.STEPPING), [])
        run.run(1.2)
        self.assertEqual(run.run(1.0, FORWARD).basic_state, p.STATE_STEPPING)

    def test_toggles_wait_for_the_state(self):
        """With reports 60 ms late, a second stepping toggle would undo the first: refused."""
        run = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True), lag=3)
        run.run(0.1)
        run.run(DT, FORWARD)  # the episode start toggles stepping
        self.assertEqual(len(run.sent_since(0.0, p.STEPPING)), 1)
        self.assertEqual(run.view().basic_state, p.STATE_TORQUE_STANDING)  # not reported yet
        self.assertFalse(run.toggle(p.STEPPING))  # e.g. start_stepping: refused
        robot = run.run(1.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertEqual(len(run.sent_since(0.0, p.STEPPING)), 1)

    def test_no_axes_without_motion_reports(self):
        robot = FakeX30(basic_state=p.STATE_STEPPING, non_manual=True)
        robot.send_motion = False
        run = Loop(robot)
        self.assertEqual(run.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertEqual(robot.codes(p.AXIS_FORWARD), [])
        self.assertEqual(run.step.ignored, 'no fresh motion state report')

    def test_gaits_without_limits_do_not_move(self):
        """Silent is not in Appendix B: nothing moves, unless gait_limits gives its speeds."""
        robot = FakeX30(basic_state=p.STATE_STEPPING, gait_state=p.GAIT_STATE_SILENT,
                        non_manual=True)
        run = Loop(robot)
        self.assertEqual(run.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertIn('silent', run.step.ignored)
        table = p.limits_table(p.parse_gait_limits({'silent': [1.2, 0.75, 0.15, 0.45]}))
        robot = FakeX30(basic_state=p.STATE_STEPPING, gait_state=p.GAIT_STATE_SILENT,
                        non_manual=True)
        run = Loop(robot, velocity=VelocityConfig(gait_limits=table))
        self.assertAlmostEqual(run.run(1.0, FORWARD).velocity[0], 0.3, places=3)

    def test_zero_twists_send_only_the_burst(self):
        """4 s of zero Twists after motion: zero axes for stop_burst_sec, then none."""
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        run.run(1.0, FORWARD)
        start = run.t
        run.run(4.0, STOP)
        sent = run.axis_datagrams(start + DT)
        self.assertEqual([axes for _, axes in sent], [(0, 0, 0)] * 25)
        self.assertLessEqual(sent[-1][0], start + 0.5 + 1e-6)

    def test_late_period_still_stops_the_robot(self):
        """cmd_vel stops and the next period runs 0.7 s late: its zeros stop the robot."""
        run = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        run.run(0.5, FORWARD)
        run.stall(0.68)
        robot = run.run(DT)
        robot.advance(run.t + 0.2)  # the robot would walk until its own 1 s axis timeout
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))


class TestEpisodeEndsOnlyOnQuiet(unittest.TestCase):
    """The same cmd_vel stream after the gate closed and opened again: no switches."""

    def episode(self):
        """Start stepping at the episode start, then drive past the toggle guard's 5 s."""
        run = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True))
        robot = run.run(6.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertEqual(len(run.sent_since(0.0, p.STEPPING)), 1)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)
        return run

    def assertNoRestart(self, run):
        """Check that the stream, armed again by a zero, toggles nothing until a quiet."""
        since = run.t
        run.run(0.1, STOP)
        robot = run.run(1.0, FORWARD)
        self.assertEqual(run.sent_since(since, p.STEPPING), [])
        self.assertEqual(robot.basic_state, p.STATE_TORQUE_STANDING)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        run.run(1.2)  # quiet: the next motion starts a new episode
        self.assertEqual(run.run(1.0, FORWARD).basic_state, p.STATE_STEPPING)

    def test_handback_after_a_takeover(self):
        run = self.episode()
        run.robot.set_mode(False)  # the operator takes over on the handheld
        run.run(0.2, FORWARD)
        self.assertIn('took over', run.stream.latch_reason)
        run.robot.handle(p.simple(p.code(p.STEPPING, p.PREFIX_REMOTE)))  # and stops stepping
        run.run(1.0, STOP)
        run.robot.set_mode(True)  # handed back
        self.assertNoRestart(run)

    def test_recovered_fall(self):
        run = self.episode()
        run.robot.basic_state = p.STATE_EMERGENCY  # a fall
        run.run(0.5, FORWARD)
        self.assertEqual(run.stream.gate(run.t), 'disarmed:emergency stop or fall')
        run.robot.basic_state = p.STATE_TORQUE_STANDING  # recovered by hand
        self.assertNoRestart(run)

    def test_stand_up_after_stop_stepping(self):
        run = self.episode()
        run.stream.hold(run.t, 'stop_stepping')  # what the service does
        self.assertTrue(run.toggle(p.STEPPING))
        run.run(1.0, STOP)
        self.assertEqual(run.robot.basic_state, p.STATE_TORQUE_STANDING)
        run.stream.release()  # stand_up: already standing
        self.assertNoRestart(run)


if __name__ == '__main__':
    unittest.main()
