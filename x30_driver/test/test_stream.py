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
    ToggleGuard,
    VelocityConfig,
    VelocityStream,
)

DT = 0.02  # cmd_rate_hz 50


class Loop:
    """The driver's send_velocity against a FakeX30; reports arrive `lag` periods late."""

    def __init__(self, robot, prefix=p.PREFIX_AUTONOMOUS, lag=0, velocity=None, **auto):
        self.robot = robot
        self.prefix = prefix
        self.stream = VelocityStream(velocity or VelocityConfig(),
                                     AutoConfig(target_non_manual=prefix == p.PREFIX_AUTONOMOUS,
                                                **auto))
        self.guard = ToggleGuard(timeout_sec=5.0)
        self.t = 0.0
        self.sent = []    # (time, command base) of every mode / toggle command
        self.step = None
        self._views = collections.deque(maxlen=lag + 1)

    def send(self, base, value=0):
        self.sent.append((self.t, base))
        self.robot.handle(p.simple(p.code(base, self.prefix), value))

    def toggle(self, base):
        """send_toggle: one stand/sit or stepping toggle until the reports show it."""
        if not self.guard.request(self.t, base, self.view().basic_state):
            return False
        self.send(base)
        return True

    def view(self):
        return self._views[0] if self._views else RobotView()

    def sent_since(self, t, base):
        return [s for s in self.sent if s[0] >= t and s[1] == base]

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
                if base == p.STEPPING:
                    self.toggle(base)
                else:
                    self.send(base)
            if self.step.axes is not None:
                for axis, value in zip(AXES, self.step.axes):
                    r.handle(p.axis(p.code(axis, self.prefix), value))
        return self.robot


FORWARD = (0.3, 0.0, 0.0)


class TestClosedLoop(unittest.TestCase):
    def test_drives_in_non_manual_mode(self):
        """0x31: non-manual mode and stepping on the way, then the commanded speed."""
        loop = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING))
        robot = loop.run(1.0, FORWARD)
        self.assertTrue(robot.non_manual)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)

    def test_handheld_takeover_is_not_reverted(self):
        """Manual on the handheld while cmd_vel drives: the driver backs off until it rests."""
        loop = Loop(FakeX30(basic_state=p.STATE_STEPPING))
        loop.run(1.0, FORWARD)
        loop.robot.set_mode(False)  # the operator switches on the handheld
        takeover = loop.t
        robot = loop.run(3.0, FORWARD)
        self.assertFalse(robot.non_manual)
        self.assertEqual(loop.sent_since(takeover, p.NON_MANUAL_MODE), [])
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        self.assertIn('took over', loop.step.ignored)
        loop.run(1.5)  # cmd_vel rests
        self.assertTrue(loop.stream.armed)
        robot = loop.run(1.0, FORWARD)
        self.assertTrue(robot.non_manual)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)

    def test_remote_prefix_drives_in_manual_mode(self):
        """0x21 (acting as the handheld): the robot obeys in manual mode, so that is the target."""
        loop = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True),
                    prefix=p.PREFIX_REMOTE)
        robot = loop.run(1.0, FORWARD)
        self.assertFalse(robot.non_manual)
        self.assertEqual(loop.sent[0][1], p.MANUAL_MODE)
        self.assertAlmostEqual(robot.velocity[0], 0.3, places=3)
        # 0x31 with the robot left in manual mode would not move it
        stuck = Loop(FakeX30(basic_state=p.STATE_STEPPING), auto_control_mode=False)
        self.assertEqual(stuck.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertIn('manual mode', stuck.step.ignored)

    def test_stop_stepping_is_not_undone(self):
        """A service stop disarms: cmd_vel that keeps coming does not start stepping again."""
        loop = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        loop.run(1.0, FORWARD)
        loop.stream.disarm(loop.t, 'stop_stepping')  # what the service does
        self.assertTrue(loop.toggle(p.STEPPING))
        stop = loop.t
        robot = loop.run(3.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_TORQUE_STANDING)
        self.assertEqual(loop.sent_since(stop + DT, p.STEPPING), [])
        loop.run(1.5)
        robot = loop.run(1.0, FORWARD)  # rested: the next motion steps again
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)

    def test_emergency_stop_holds_back(self):
        loop = Loop(FakeX30(basic_state=p.STATE_STEPPING, non_manual=True))
        loop.run(0.5, FORWARD)
        loop.robot.handle(p.simple(p.code(p.SOFT_ESTOP)))
        estop = loop.t
        loop.run(3.0, FORWARD)
        self.assertEqual(loop.sent_since(estop, p.STEPPING), [])
        self.assertFalse(loop.stream.armed)
        self.assertIn('emergency', loop.step.ignored)

    def test_toggles_wait_for_the_state(self):
        """With reports 60 ms late, a second stepping toggle would undo the first: refused."""
        loop = Loop(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True), lag=3)
        loop.run(0.1)
        loop.run(DT, FORWARD)  # the automatic switch toggles stepping
        self.assertEqual(len(loop.sent_since(0.0, p.STEPPING)), 1)
        self.assertEqual(loop.view().basic_state, p.STATE_TORQUE_STANDING)  # not reported yet
        self.assertFalse(loop.toggle(p.STEPPING))  # e.g. start_stepping: refused
        robot = loop.run(1.0, FORWARD)
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        self.assertEqual(len(loop.sent_since(0.0, p.STEPPING)), 1)

    def test_no_axes_without_motion_reports(self):
        robot = FakeX30(basic_state=p.STATE_STEPPING, non_manual=True)
        robot.send_motion = False
        loop = Loop(robot)
        self.assertEqual(loop.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertEqual(robot.codes(p.AXIS_FORWARD), [])
        self.assertEqual(loop.step.ignored, 'no fresh motion state report')

    def test_gaits_without_limits_do_not_move(self):
        """Silent is not in Appendix B: zeros, unless gait_limits gives its speeds."""
        robot = FakeX30(basic_state=p.STATE_STEPPING, gait_state=p.GAIT_STATE_SILENT,
                        non_manual=True)
        loop = Loop(robot)
        self.assertEqual(loop.run(1.0, FORWARD).velocity, (0.0, 0.0, 0.0))
        self.assertIn('silent', loop.step.ignored)
        table = p.limits_table(p.parse_gait_limits({'silent': [1.2, 0.75, 0.15, 0.45]}))
        robot = FakeX30(basic_state=p.STATE_STEPPING, gait_state=p.GAIT_STATE_SILENT,
                        non_manual=True)
        loop = Loop(robot, velocity=VelocityConfig(gait_limits=table))
        self.assertAlmostEqual(loop.run(1.0, FORWARD).velocity[0], 0.3, places=3)


if __name__ == '__main__':
    unittest.main()
