#!/usr/bin/env python3
"""The simulated robot behaves as the document describes the motion and perception hosts."""

import os
import queue
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, HERE)  # nose -P
from fake_robot import FakePerception, FakeRobotServer, FakeX30  # noqa: E402
from x30_driver import protocol as p  # noqa: E402
from x30_driver.link import UdpLink  # noqa: E402


def cmd(base, value=0, prefix=p.PREFIX_AUTONOMOUS):
    return p.simple(p.code(base, prefix), value)


class TestFakeX30(unittest.TestCase):
    def setUp(self):
        self.robot = FakeX30()

    def step(self, dt):
        self.robot.advance(self.robot.now + dt)

    def test_stand_up_and_sit_down(self):
        """Stand/sit alternates through the transition states (1.2.2)."""
        self.robot.handle(cmd(p.STAND_SIT))
        self.assertEqual(self.robot.basic_state, p.STATE_SIT_TO_STAND)
        self.step(1.0)
        self.assertEqual(self.robot.basic_state, p.STATE_INITIAL_STANDING)
        self.robot.handle(cmd(p.TORQUE_CONTROL))
        self.assertEqual(self.robot.basic_state, p.STATE_TORQUE_STANDING)
        self.robot.handle(cmd(p.STAND_SIT))
        self.assertEqual(self.robot.basic_state, p.STATE_STAND_TO_SIT)
        self.step(1.0)
        self.assertEqual(self.robot.basic_state, p.STATE_SITTING)

    def test_stepping_toggle_and_gaits(self):
        """Gaits change in the stepping state only (1.2.6)."""
        robot = FakeX30(basic_state=p.STATE_TORQUE_STANDING)
        robot.handle(cmd(p.GAIT_SLOPE))
        self.assertEqual(robot.gait_state, p.GAIT_STATE_WALK)
        robot.handle(cmd(p.STEPPING))
        self.assertEqual(robot.basic_state, p.STATE_STEPPING)
        robot.handle(cmd(p.GAIT_MOUNTAIN))
        self.assertEqual(robot.gait_state, p.GAIT_STATE_MOUNTAIN)
        robot.handle(cmd(p.STEPPING))
        self.assertEqual(robot.basic_state, p.STATE_TORQUE_STANDING)

    def test_axes_follow_the_control_mode(self):
        """Stepping only; 0x21 axes in manual mode, 0x31 axes in non-manual mode."""
        robot = FakeX30(basic_state=p.STATE_STEPPING)            # manual mode
        robot.handle(cmd(p.AXIS_FORWARD, 16384))                 # 0x31: ignored in manual
        robot.advance(0.1)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        robot.handle(cmd(p.AXIS_FORWARD, 16384, prefix=p.PREFIX_REMOTE))
        robot.advance(0.2)
        self.assertAlmostEqual(robot.velocity[0], 0.6, places=3)
        robot.handle(cmd(p.NON_MANUAL_MODE))                     # the axes start over
        robot.advance(0.3)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        robot.handle(cmd(p.AXIS_FORWARD, 16384, prefix=p.PREFIX_REMOTE))  # 0x21 now ignored
        robot.handle(cmd(p.AXIS_TURN, -32767))
        robot.advance(1.0)
        self.assertEqual(robot.velocity[0], 0.0)
        self.assertAlmostEqual(robot.velocity[2], 0.45)  # turn axis -: counter-clockwise
        self.assertGreater(robot.pose[0], 0.0)
        robot.advance(2.5)  # 1.2.3: no axis command for more than a second
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))
        loose = FakeX30(basic_state=p.STATE_STEPPING, require_mode=False)
        loose.handle(cmd(p.AXIS_FORWARD, 16384))
        loose.advance(0.1)
        self.assertGreater(loose.velocity[0], 0.0)

    def test_axes_ignored_when_not_stepping(self):
        robot = FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True)
        robot.handle(cmd(p.AXIS_FORWARD, 30000))
        robot.advance(0.5)
        self.assertEqual(robot.velocity, (0.0, 0.0, 0.0))

    def test_height_estop_save_and_codes(self):
        self.robot.handle(cmd(p.BODY_HEIGHT, p.HEIGHT_CRAWL))
        self.assertEqual(p.signed(p.parse_datagram(self.robot.height_report()).value), -1)
        self.robot.handle(cmd(p.HEARTBEAT))
        self.robot.handle(cmd(p.CONNECT))
        self.assertTrue(self.robot.connected)
        self.assertEqual(self.robot.heartbeats, 1)
        self.robot.handle(cmd(p.SOFT_ESTOP))
        self.robot.handle(cmd(p.SOFT_ESTOP))
        self.assertEqual((self.robot.basic_state, self.robot.estops), (p.STATE_EMERGENCY, 2))
        rcs = p.decode_rcs(p.parse_datagram(self.robot.rcs_report()).payload)
        self.assertEqual(p.name_of(p.EMERGENCY_SOURCES, rcs.emergency_source), 'client')
        self.robot.handle(p.simple(p.save_data_command(p.SAVE_DATA)))
        self.assertEqual(self.robot.saved, 0x31010C01)
        self.robot.handle(p.simple(p.SAVE_DATA_OLD))
        self.assertEqual(self.robot.saved, 18)
        self.assertEqual(self.robot.codes(p.HEARTBEAT), [0x31040001])
        frozen = FakeX30()
        frozen.ignore_height = True
        frozen.handle(cmd(p.BODY_HEIGHT, p.HEIGHT_CRAWL))
        self.assertEqual(frozen.height, 0)

    def test_reports_decode(self):
        """Every report is a datagram the driver's decoders read."""
        robot = FakeX30(basic_state=p.STATE_STEPPING, gait_state=p.GAIT_STATE_SLOPE)
        motion = p.decode_motion_state(p.parse_datagram(robot.motion_report()).payload)
        self.assertEqual((motion.basic_state, motion.gait_state), (4, 2))
        sensor = p.decode_sensor(p.parse_datagram(robot.sensor_report()).payload)
        self.assertEqual(sensor.rpy_deg[:2], (0.5, -1.0))
        battery = p.decode_battery(p.parse_datagram(robot.battery_report()).payload)
        self.assertEqual(battery.battery_level, 87)
        self.assertIsNotNone(p.decode_safe(p.parse_datagram(robot.safe_report()).payload))
        packed = FakeX30(layout='pack1')
        self.assertEqual(p.decode_rcs(p.parse_datagram(packed.rcs_report()).payload).layout,
                         'pack1')


class TestFakePerception(unittest.TestCase):
    def test_elevation_map(self):
        """Any message starts the feedback; modes come back as 0x3100EE0x (1.5)."""
        host = FakePerception()
        self.assertFalse(host.feedback_on)
        self.assertEqual(host.handle(p.simple(p.ELEVATION_QUERY), 'elevation'), [])
        self.assertTrue(host.feedback_on)
        host.handle(p.simple(p.ELEVATION_MODE, p.ELEVATION_GRID_GROUND), 'elevation')
        host.handle(p.simple(p.BRAKE_MODE, 2), 'elevation')
        host.handle(p.simple(p.ELEVATION_MODE, 99), 'elevation')  # not a mode: ignored
        host.handle(p.simple(p.OBSTACLE_HEIGHT_LIKELY, 2), 'elevation')
        feedback = {d.code: d.value for d in map(p.parse_datagram, host.feedback())}
        self.assertEqual(feedback, {p.ELEVATION_MODE_STATE: 4, p.BRAKE_MODE_STATE: 2,
                                    p.VELOCITY_SOURCE_STATE: 2})
        self.assertEqual((host.obstacle_height, host.obstacle_codes), (2, [0x3101EE04]))

    def test_multi_frame_and_lio(self):
        """Multi-frame needs the odometry on; it turns off only after leaving (1.5.2.4/5)."""
        host = FakePerception()

        def lio(value):
            reply = p.parse_datagram(host.handle(p.simple(p.LIO, value), 'lio')[0])
            return reply.code, p.signed(reply.value)

        host.handle(p.simple(p.ELEVATION_MODE, 20), 'elevation')
        self.assertEqual(host.elevation_mode, 18)  # odometry off: stuck preparing
        host.advance(5.0)
        self.assertEqual(host.elevation_mode, 18)
        self.assertEqual(lio(p.LIO_ON), (p.LIO, 0))
        host.handle(p.simple(p.ELEVATION_MODE, 20), 'elevation')
        host.advance(6.0)
        self.assertEqual(host.elevation_mode, 20)
        self.assertEqual(lio(p.LIO_OFF), (p.LIO, -1))  # still in multi-frame
        host.handle(p.simple(p.ELEVATION_MODE, 3), 'elevation')
        self.assertEqual(lio(p.LIO_OFF), (p.LIO, 0))
        self.assertFalse(host.lio_on)

    def test_charging(self):
        """Request / response with charge_manager_state (1.4)."""
        host = FakePerception()
        reply = p.parse_datagram(host.handle(p.simple(p.CHARGE, p.CHARGE_START), 'charge')[0])
        self.assertEqual((reply.code, reply.value), (p.CHARGE, 1))
        host.advance(1.0)
        reply = p.parse_datagram(host.handle(p.simple(p.CHARGE_QUERY), 'charge')[0])
        self.assertEqual((reply.code, reply.value), (p.CHARGE_QUERY, 2))
        host.handle(p.simple(p.CHARGE, p.CHARGE_CLEAR), 'charge')
        self.assertEqual(host.charge_state, 0)


class TestServer(unittest.TestCase):
    def test_reports_and_commands_over_udp(self):
        """The server reports to the configured address and runs the commands it receives."""
        inbox = queue.Queue()
        client = UdpLink('127.0.0.1', 9, '127.0.0.1', 0, lambda d, a: inbox.put(d))
        client.start()
        robot = FakeX30()
        server = FakeRobotServer(robot, FakePerception(), motion_port=0,
                                 report_to=client.local, vmap_port=0, charge_port=0,
                                 lio_port=0, rate_hz=100.0).start()
        try:
            codes = set()
            end = time.monotonic() + 3.0
            while time.monotonic() < end and len(codes) < 6:
                codes.add(p.parse_datagram(inbox.get(timeout=1.0)).code)
            self.assertEqual(codes, {p.RCS_DATA, p.MOTION_STATE, p.SENSOR_DATA, p.SAFE_DATA,
                                     p.BATTERY, p.BODY_HEIGHT_STATUS})
            client.send(cmd(p.STAND_SIT), ('127.0.0.1', server.motion_port))
            end = time.monotonic() + 3.0
            while time.monotonic() < end and robot.basic_state != p.STATE_INITIAL_STANDING:
                time.sleep(0.02)
            self.assertEqual(robot.basic_state, p.STATE_INITIAL_STANDING)
            client.send(p.simple(p.CHARGE_QUERY), ('127.0.0.1', server.charge_port))
            end = time.monotonic() + 3.0
            reply = None
            while time.monotonic() < end and reply is None:
                d = p.parse_datagram(inbox.get(timeout=1.0))
                reply = d if d.code == p.CHARGE_QUERY else None
            self.assertEqual(reply.value, 0)
        finally:
            server.stop()
            client.stop()

    def test_perception_ports(self):
        """Every elevation map client gets the feedback; the odometry port answers."""
        inboxes = [queue.Queue(), queue.Queue()]
        clients = [UdpLink('127.0.0.1', 9, '127.0.0.1', 0, lambda d, a, q=q: q.put(d))
                   for q in inboxes]
        for client in clients:
            client.start()
        server = FakeRobotServer(FakeX30(), FakePerception(), motion_port=0,
                                 report_to=('127.0.0.1', 9), vmap_port=0, charge_port=0,
                                 lio_port=0, rate_hz=100.0).start()

        def next_code(inbox, code):
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                d = p.parse_datagram(inbox.get(timeout=1.0))
                if d.code == code:
                    return d
            return None

        try:
            for client in clients:
                client.send(p.simple(p.ELEVATION_QUERY), ('127.0.0.1', server.vmap_port))
            for inbox in inboxes:
                self.assertEqual(next_code(inbox, p.ELEVATION_MODE_STATE).value, 3)
            clients[0].send(p.simple(p.LIO, p.LIO_ON), ('127.0.0.1', server.lio_port))
            self.assertEqual(next_code(inboxes[0], p.LIO).value, 0)
        finally:
            server.stop()
            for client in clients:
                client.stop()


if __name__ == '__main__':
    unittest.main()
