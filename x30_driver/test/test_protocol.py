#!/usr/bin/env python3
"""Codec tests against the Jueying X30 API Documentation V1.0.5-0."""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from x30_driver import protocol as p  # noqa: E402

# Appendix A: a joint-angle datagram as the document prints it, 16-bit words in byte order.
APPENDIX_A = (
    '0209 0000 6000 0000 0100 0000 2efd 3ccf 47e4 e1bf f130 4992 f49e e7bf c3d7 eefe '
    '8567 f13f 9ae4 b66d 6583 f1bf 24d8 f33c 3d00 f13f 8c16 222a 0b1f 0140 e8ba aaaa '
    '3803 ef3f d009 0000 60bd d2bf db04 3353 1150 e73f 7d0b 0000 72f0 e53f 0a3d 0cc3 '
    'f071 e73f c3d7 eefe 7b52 f93f'
)
APPENDIX_A_ANGLES = [
    -0.5591162726995316, -0.7381537301203293, 1.0877742727584383,
    -1.0945791516990426, 1.0625584011991203, 2.1401580135014395,
    0.9691432317102597, -0.2928085327149832, 0.7285238862024487,
    0.6856012344363617, 0.7326587495353192, 1.5826377828902742,
]

# Every command code the document prints (all with the remote prefix 0x21), by section.
DOCUMENT_CODES = {
    p.HEARTBEAT: 0x21040001,          # 1.2.1
    p.CONNECT: 0x21020001,            # 1.2.1
    p.STAND_SIT: 0x21010202,          # 1.2.2
    p.TORQUE_CONTROL: 0x2101020A,     # 1.2.2
    p.STEPPING: 0x21010201,           # 1.2.2
    p.AXIS_LEFT_X: 0x21010131,        # 1.2.3
    p.AXIS_LEFT_Y: 0x21010130,
    p.AXIS_RIGHT_X: 0x21010135,
    p.AXIS_RIGHT_Y: 0x21010102,
    p.NON_MANUAL_MODE: 0x21010C03,    # 1.2.4
    p.MANUAL_MODE: 0x21010C02,
    p.BODY_HEIGHT: 0x21010406,        # 1.2.5
    p.GAIT_WALK: 0x21010300,          # 1.2.6
    p.GAIT_SLOPE: 0x21010402,
    p.GAIT_OFF_ROAD: 0x21010401,
    p.GAIT_STAIR: 0x21010405,
    p.GAIT_STAIR_MULTI_FRAME: 0x2101040A,
    p.GAIT_STAIR_45: 0x2101040B,
    p.GAIT_L_WALK: 0x21010420,
    p.GAIT_MOUNTAIN: 0x21010421,
    p.GAIT_SILENT: 0x21010422,
    p.SOFT_ESTOP: 0x21010C0E,         # 1.2.8
}


def rcs(**overrides):
    """Return RcsData with distinct values in every field."""
    values = dict(robot_name='X30-0042', current_mileage=1234, total_mileage=-5,
                  current_run_time=3600, total_run_time=2 ** 40, current_motion_time=1800,
                  total_motion_time=7, joystick=(0.25, -0.5, 0.75, -1.0), is_nav_mode=1,
                  emergency_source=5, error_state=0b10_0010_0001)
    values.update(overrides)
    return p.RcsData(**values)


def motion(**overrides):
    """Return MotionStateData with distinct values in every field."""
    values = dict(basic_state=4, gait_state=33, leg_odom_pos=(1.5, -2.25, 0.5),
                  leg_odom_vel=(0.75, -0.125, 0.25), robot_distance=321.5, touch_state=15,
                  control_state=6, auto_charge_state=1, pos_ctrl_state=2,
                  max_forward_vel=1.25, max_backward_vel=0.5)
    values.update(overrides)
    return p.MotionState(**values)


def battery(**overrides):
    """Return BatterySensorData with distinct values in every field."""
    values = dict(voltage=5432, current=-1234, remaining_capacity=1200, nominal_capacity=1500,
                  cycles=42, production_date=0x2408, balanced_low=3, balanced_high=4,
                  protected_state=0x10, software_version=7, battery_level=87, mos_state=3,
                  battery_quantity=1, battery_ntc=4, temperatures=(28.0, 29.5, 30.25, 31.0))
    values.update(overrides)
    return p.BatteryData(**values)


class TestHead(unittest.TestCase):
    def test_appendix_a(self):
        """The document's own worked example: head, length and 12 doubles."""
        raw = bytes.fromhex(APPENDIX_A.replace(' ', ''))
        self.assertEqual(len(raw), 108)
        d = p.parse_datagram(raw)
        self.assertEqual((d.code, d.kind, d.value), (0x0902, p.TYPE_COMPLEX, 96))
        self.assertEqual(list(struct.unpack('<12d', d.payload)), APPENDIX_A_ANGLES)

    def test_simple_command_bytes(self):
        """A simple command is code, value, type 0, little-endian (1.1.1)."""
        self.assertEqual(p.simple(p.code(p.HEARTBEAT, p.PREFIX_REMOTE)),
                         bytes.fromhex('01000421' '00000000' '00000000'))
        self.assertEqual(p.simple(p.code(p.HEARTBEAT)), bytes.fromhex('01000431' + '00' * 8))
        self.assertEqual(p.simple(p.BODY_HEIGHT_STATUS, -1)[4:8], b'\xff\xff\xff\xff')
        self.assertEqual(p.signed(0xFFFFFFFF), -1)

    def test_document_codes(self):
        """code() with the remote prefix gives every code the document prints."""
        for base, full in DOCUMENT_CODES.items():
            self.assertEqual(p.code(base, p.PREFIX_REMOTE), full, hex(full))
            self.assertEqual(p.code(base, p.PREFIX_AUTONOMOUS), 0x31000000 | (full & 0xFFFFFF))
        self.assertEqual(p.code(p.HEARTBEAT), 0x31040001)  # the document's 0x31 example
        self.assertEqual((p.SAVE_DATA, p.SAVE_DATA_OLD), (0x010C01, 18))  # 1.2.7, no prefix

    def test_report_codes(self):
        """1.3 report codes."""
        self.assertEqual((p.RCS_DATA, p.MOTION_STATE, p.SENSOR_DATA, p.SAFE_DATA),
                         (0x1008, 0x1009, 0x100A, 0x100B))
        self.assertEqual((p.BATTERY, p.BODY_HEIGHT_STATUS), (0x21050F0A, 0x11050F08))

    def test_prefix_and_heartbeat_limits(self):
        """Only the document's two source bytes; heartbeat at least 2 Hz (1.2, 1.2.1)."""
        self.assertEqual(p.check_prefix(0x21), 0x21)
        self.assertEqual(p.check_prefix('49'), 0x31)
        for bad in (0x00, 0x11, 0x41, 0x2131):
            with self.assertRaises(ValueError):
                p.check_prefix(bad)
        self.assertEqual(p.heartbeat_rate(0.5), 2.0)
        self.assertEqual(p.heartbeat_rate(5), 5.0)

    def test_save_data_code(self):
        """The new save data code carries the source byte; the old raw 18 does not."""
        self.assertEqual(p.save_data_command(p.SAVE_DATA), 0x31010C01)
        self.assertEqual(p.save_data_command(p.SAVE_DATA, p.PREFIX_REMOTE), 0x21010C01)
        self.assertEqual(p.save_data_command(p.SAVE_DATA_OLD), 18)
        self.assertEqual(p.save_data_command(0x21010C01, p.PREFIX_AUTONOMOUS), 0x31010C01)

    def test_malformed(self):
        """Short heads, unknown types and truncated payloads are dropped."""
        self.assertIsNone(p.parse_datagram(b'\x01\x02'))
        self.assertIsNone(p.parse_datagram(struct.pack('<III', p.MOTION_STATE, 0, 7)))
        self.assertIsNone(p.parse_datagram(struct.pack('<III', p.MOTION_STATE, 60, 1) + b'\0'))
        self.assertIsNone(p.parse_datagram(struct.pack('<III', p.MOTION_STATE, 5000, 1)))
        status = p.parse_datagram(p.simple(p.BODY_HEIGHT_STATUS, -1))
        self.assertEqual((status.kind, p.signed(status.value)), (p.TYPE_SIMPLE, -1))


class TestStructs(unittest.TestCase):
    def test_sizes(self):
        """Struct sizes: natural alignment, pack(1) and a 4-byte long."""
        sizes = {name: s.size for name, s in p.RCS_LAYOUTS.items()}
        self.assertEqual(sizes, {'natural': 88, 'pack1': 85, 'ilp32': 72, 'ilp32_pack1': 69})
        self.assertEqual({n: s.size for n, s in p.MOTION_LAYOUTS.items()},
                         {'natural': 60, 'pack1': 56})
        self.assertEqual(p.SENSOR_LAYOUT.size, 184)  # (4 + 9 * 4) + 3 * 12 * 4
        self.assertEqual(p.SAFE_LAYOUT.size, 68)     # 12 * 4 + 12 + 2 * 4
        self.assertEqual({n: s.size for n, s in p.BATTERY_LAYOUTS.items()},
                         {'natural': 40, 'pack1': 39})

    def test_rcs_round_trip_and_offsets(self):
        """RcsData in every layout; natural offsets as a C compiler lays them out."""
        for layout in p.RCS_LAYOUTS:
            with self.subTest(layout=layout):
                sent = rcs(layout=layout,
                           total_run_time=2 ** 20 if layout.startswith('ilp32') else 2 ** 40)
                d = p.parse_datagram(p.encode_rcs(sent, layout))
                self.assertEqual(d.code, p.RCS_DATA)
                self.assertEqual(p.decode_rcs(d.payload), sent)
        payload = p.encode_rcs(rcs())[12:]
        self.assertEqual(payload[:8], b'X30-0042')
        self.assertEqual(struct.unpack_from('<ii', payload, 16), (1234, -5))
        self.assertEqual(struct.unpack_from('<q', payload, 24)[0], 3600)
        self.assertEqual(struct.unpack_from('<4f', payload, 56), (0.25, -0.5, 0.75, -1.0))
        self.assertEqual(payload[72:74], b'\x01\x05')  # is_nav_mode, emergency_source
        self.assertEqual(struct.unpack_from('<I', payload, 84)[0], 0b10_0010_0001)
        got = p.decode_rcs(payload)
        self.assertTrue(got.non_manual)
        self.assertEqual(got.errors, ('imu_timeout', 'low_battery', 'cpu_frequency_low'))
        self.assertIsNone(p.decode_rcs(b'\0' * 80))

    def test_motion_round_trip_and_offsets(self):
        """MotionStateData: natural offsets and the pack(1) layout."""
        for layout in p.MOTION_LAYOUTS:
            with self.subTest(layout=layout):
                sent = motion(layout=layout)
                payload = p.encode_motion_state(sent, layout)[12:]
                self.assertEqual(p.decode_motion_state(payload), sent)
        payload = p.encode_motion_state(motion())[12:]
        self.assertEqual(payload[:2], b'\x04\x21')
        self.assertEqual(struct.unpack_from('<2f', payload, 4), (1.25, 0.5))
        self.assertEqual(struct.unpack_from('<3f', payload, 12), (1.5, -2.25, 0.5))
        self.assertEqual(struct.unpack_from('<3f', payload, 24), (0.75, -0.125, 0.25))
        self.assertEqual(struct.unpack_from('<fII', payload, 36), (321.5, 15, 6))
        self.assertEqual(payload[48:50], b'\x01\x02')  # auto_charge_state, pos_ctrl_state
        self.assertTrue(p.decode_motion_state(payload).stepping)
        self.assertIsNone(p.decode_motion_state(b'\0' * 58))

    def test_sensor_round_trip(self):
        """ControllerSensorData: IMU then positions, velocities, torques in joint order."""
        sent = p.SensorData(
            timestamp=-7, rpy_deg=(1.5, -2.5, 90.0), omega=(0.125, 0.25, -0.5),
            acc=(0.0, 0.5, 9.75), joint_pos=tuple(i * 0.125 for i in range(12)),
            joint_vel=tuple(-i * 0.25 for i in range(12)),
            joint_tau=tuple(i * 1.5 for i in range(12)))
        payload = p.encode_sensor(sent)[12:]
        self.assertEqual(len(payload), 184)
        self.assertEqual(p.decode_sensor(payload), sent)
        self.assertEqual(struct.unpack_from('<3f', payload, 4), (1.5, -2.5, 90.0))  # rpy
        self.assertEqual(struct.unpack_from('<f', payload, 40 + 4)[0], 0.125)  # fl_hipy pos
        self.assertEqual(struct.unpack_from('<f', payload, 88 + 4)[0], -0.25)  # fl_hipy vel
        self.assertEqual(struct.unpack_from('<f', payload, 136 + 4)[0], 1.5)   # fl_hipy tau
        self.assertIsNone(p.decode_sensor(b'\0' * 100))

    def test_safe_round_trip(self):
        """ControllerSafeData: motor floats, driver bytes, CPU info."""
        sent = p.SafeData(motor_temperature=tuple(30.0 + i for i in range(12)),
                          driver_temperature=tuple(40 + i for i in range(12)),
                          cpu_temperature=55.5, cpu_frequency=1800.0)
        payload = p.encode_safe(sent)[12:]
        self.assertEqual(p.decode_safe(payload), sent)
        self.assertEqual(struct.unpack_from('<2f', payload, 60), (55.5, 1800.0))

    def test_battery_round_trip_and_offsets(self):
        """BatterySensorData: natural (40) and pack(1) (39)."""
        for layout in p.BATTERY_LAYOUTS:
            with self.subTest(layout=layout):
                sent = battery(layout=layout)
                payload = p.encode_battery(sent, layout)[12:]
                self.assertEqual(p.decode_battery(payload), sent)
        payload = p.encode_battery(battery())[12:]
        self.assertEqual(struct.unpack_from('<Hh', payload, 0), (5432, -1234))
        self.assertEqual(payload[19], 87)  # battery_level
        self.assertEqual(struct.unpack_from('<f', payload, 24)[0], 28.0)
        self.assertIsNone(p.decode_battery(b'\0' * 30))


class TestTables(unittest.TestCase):
    def test_states(self):
        """1.3.2 [4] basic states and [5] gaits."""
        self.assertEqual(p.BASIC_STATES[3], 'torque_standing')
        self.assertEqual(p.BASIC_STATES[6], 'emergency_stop_or_fall')
        self.assertEqual(sorted(p.GAIT_STATES), [0, 1, 2, 3, 6, 7, 8, 32, 33, 34])
        self.assertEqual(p.name_of(p.GAIT_STATES, 33), 'mountain')
        self.assertEqual(p.name_of(p.BASIC_STATES, 99), 'unknown')
        self.assertEqual(p.name_of(p.BASIC_STATES, None), 'unknown')

    def test_gait_commands_report_their_gait(self):
        """Every gait command maps to the gait state the robot then reports."""
        self.assertEqual(len(p.GAIT_COMMANDS), 9)
        self.assertEqual(p.GAIT_COMMANDS[p.GAIT_SLOPE], 2)
        self.assertEqual(p.GAIT_COMMANDS[p.GAIT_OFF_ROAD], 1)
        self.assertEqual(p.GAIT_COMMANDS[p.GAIT_SILENT], 34)

    def test_error_bits(self):
        """RcsData error_state bits from the lowest (1.3.1 [3])."""
        self.assertEqual(p.ERROR_BITS[0], 'imu_timeout')
        self.assertEqual(p.ERROR_BITS[1], 'heartbeat_timeout')
        self.assertEqual(p.ERROR_BITS[9], 'cpu_frequency_low')
        self.assertEqual(len(p.ERROR_BITS), 10)

    def test_perception_codes(self):
        """1.4 and 1.5 codes and values."""
        self.assertEqual((p.ELEVATION_MODE, p.ELEVATION_MODE_STATE), (0x3101EE01, 0x3100EE01))
        self.assertEqual((p.BRAKE_MODE, p.VELOCITY_SOURCE), (0x3101EE02, 0x3101EE03))
        self.assertEqual((p.OBSTACLE_HEIGHT, p.ELEVATION_QUERY, p.LIO),
                         (0x3100EE04, 0x3101EEFF, 0x0BAA0001))
        self.assertEqual((p.CHARGE, p.CHARGE_QUERY), (0x91910250, 0x91910253))
        self.assertEqual(p.CHARGE_STATES[0x100A], 'no_current_leaving_dock')
        self.assertEqual(sorted(p.ELEVATION_MODES), [3, 4, 5, 18, 20])
        self.assertEqual((p.ELEVATION_MAP_PORT, p.CHARGE_PORT, p.LIO_PORT), (43899, 3333, 60000))

    def test_joint_names(self):
        """12 joints in the document's order, named as in Deep Robotics' URDF."""
        self.assertEqual(len(p.JOINT_NAMES), 12)
        self.assertEqual(p.JOINT_NAMES[:3], ('FL_HipX_joint', 'FL_HipY_joint', 'FL_Knee_joint'))
        self.assertEqual(p.JOINT_NAMES[-1], 'HR_Knee_joint')


class TestVelocity(unittest.TestCase):
    def test_appendix_b(self):
        """Gait speeds of Appendix B; crawl halves walk and slope forward."""
        self.assertEqual(p.gait_limits(0), p.GaitLimits(1.2, 0.75, 0.15, 0.45))
        self.assertEqual(p.gait_limits(2), p.GaitLimits(0.7, 0.75, 0.25, 0.5))
        self.assertEqual(p.gait_limits(1), p.GaitLimits(0.3, 0.4, 0.1, 0.5))
        self.assertEqual(p.gait_limits(6), p.GaitLimits(0.3, 0.3, 0.2, 0.8))
        self.assertEqual(p.gait_limits(7), p.GaitLimits(0.6, 0.3, 0.2, 0.8))
        self.assertEqual(p.gait_limits(8), p.GaitLimits(0.3, 0.2, 0.2, 0.8))
        self.assertEqual(p.gait_limits(32), p.GaitLimits(1.0, 1.0, 0.5, 1.2))
        self.assertEqual(p.gait_limits(33), p.GaitLimits(1.0, 1.0, 0.8, 1.2))
        self.assertEqual(p.gait_limits(0, crawl=True).forward, 0.6)
        self.assertEqual(p.gait_limits(2, crawl=True).forward, 0.35)
        self.assertEqual(p.gait_limits(1, crawl=True).forward, 0.3)
        self.assertIsNone(p.gait_limits(p.GAIT_STATE_RUN))       # not in Appendix B
        self.assertIsNone(p.gait_limits(p.GAIT_STATE_SILENT))
        self.assertIsNone(p.gait_limits(None))
        self.assertFalse(p.known_gait(34))

    def test_gait_limits_parameter(self):
        """gait_limits gives Run / Silent (or overrides) by name or number; bad input raises."""
        table = p.limits_table(p.parse_gait_limits(
            {'silent': [0.5, 0.4, 0.2, 0.6], 3: (2.0, 1.0, 0.5, 1.5),
             'walk': [1.0, 0.5, 0.1, 0.4]}))
        self.assertEqual(p.gait_limits(34, table=table), p.GaitLimits(0.5, 0.4, 0.2, 0.6))
        self.assertEqual(p.gait_limits(3, table=table), p.GaitLimits(2.0, 1.0, 0.5, 1.5))
        self.assertEqual(p.gait_limits(0, crawl=True, table=table).forward, 0.5)
        self.assertEqual(p.gait_limits(33, table=table), p.GAIT_LIMITS[33])  # untouched
        self.assertTrue(p.known_gait(34, table))
        self.assertEqual(p.parse_gait_limits({}), {})
        self.assertEqual(p.parse_gait_limits(None), {})
        for bad in ({'sprint': [1, 1, 1, 1]}, {'silent': [1, 1, 1]}, {'silent': [1, 1, 1, 0]},
                    {'run': [1, 1, float('nan'), 1]}, {'run': 'fast'}, [1, 2, 3, 4]):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    p.parse_gait_limits(bad)

    def test_signs(self):
        """REP-103 in, robot axes out: forward +, left -> lateral -, ccw -> turn -."""
        walk = p.gait_limits(0)
        self.assertEqual(p.velocity_to_axes(1.2, 0.0, 0.0, walk), (32767, 0, 0))
        self.assertEqual(p.velocity_to_axes(-0.75, 0.0, 0.0, walk), (-32767, 0, 0))
        self.assertEqual(p.velocity_to_axes(0.0, 0.15, 0.0, walk), (0, -32767, 0))
        self.assertEqual(p.velocity_to_axes(0.0, 0.0, 0.45, walk), (0, 0, -32767))
        self.assertEqual(p.velocity_to_axes(0.0, 0.0, -0.45, walk), (0, 0, 32767))
        self.assertEqual(p.velocity_to_axes(0.6, 0.0, 0.0, walk)[0], 16384)

    def test_saturation_and_dead_zone(self):
        """Above the gait maximum the axis saturates; tiny commands go out just past 655."""
        walk = p.gait_limits(0)
        self.assertEqual(p.velocity_to_axes(5.0, -5.0, 5.0, walk), (32767, 32767, -32767))
        self.assertEqual(p.velocity_to_axes(0.01, 0.0, 0.0, walk), (656, 0, 0))
        self.assertEqual(p.velocity_to_axes(-0.01, 0.0, 0.0, walk), (-656, 0, 0))
        self.assertEqual(p.axis_value(655 / 32767 * 1.2, 1.2), 656)  # on the edge: still out
        self.assertEqual(p.velocity_to_axes(0.01, 0.0, 0.0, walk, dead_zone=0), (273, 0, 0))
        # noise below VELOCITY_EPSILON is 0, never a dead zone bump (no creep)
        self.assertEqual(p.velocity_to_axes(5e-4, -9e-4, 1e-4, walk), (0, 0, 0))
        self.assertEqual(p.axis_value(-0.002, 1.2), -656)
        self.assertEqual(p.axis_value(float('nan'), 1.0), 0)
        self.assertEqual(p.axis_value(1.0, 0.0), 0)

    def test_round_trip_through_the_document_formulas(self):
        """The robot's formulas (1.2.3.2) give back the commanded velocity."""
        for gait in (0, 2, 6, 33):
            limits = p.gait_limits(gait)
            for cmd in ((0.2, 0.05, 0.3), (-0.1, -0.05, -0.2), (0.25, 0.0, 0.0)):
                with self.subTest(gait=gait, cmd=cmd):
                    axes = p.velocity_to_axes(*cmd, limits)
                    got = p.axes_to_velocity(*axes, limits)
                    for want, have, top in zip(cmd, got, (limits.forward, limits.lateral,
                                                          limits.yaw)):
                        self.assertAlmostEqual(want, have, delta=top / p.AXIS_MAX + 1e-9)
        self.assertEqual(p.axes_to_velocity(654, -654, 600, p.gait_limits(0)), (0.0, 0.0, 0.0))

    def test_axis_encoding(self):
        """Axis commands are clamped simple commands."""
        data = p.axis(p.code(p.AXIS_RIGHT_X), -50000)
        self.assertEqual(p.signed(struct.unpack('<III', data)[1]), -32767)
        self.assertEqual(struct.unpack('<III', p.axis(p.code(p.AXIS_LEFT_Y), 1000)),
                         (0x31010130, 1000, 0))
        self.assertEqual(p.clamp_axis(float('inf')), 0)
        self.assertTrue(math.isclose(p.AXIS_MAX, 32767))


if __name__ == '__main__':
    unittest.main()
