"""
Codec of the Jueying X30 motion host's UDP protocol.

Source: Deep Robotics "Jueying X30 API Documentation (beta)" V1.0.5-0, 2025-08-21, sections 1.1
to 1.5 and appendices A and B. Every datagram starts with a little-endian CommandHead
{uint32 code, uint32 paramters_size, uint32 type}: simple commands (type 0) carry their value in
paramters_size, complex commands (type 1) carry paramters_size bytes of data. No ROS imports and
no sockets: unit-tested on its own and shared with the simulated robot.
"""

import math
import struct
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

HEAD = struct.Struct('<III')
HEAD_SIZE = HEAD.size
TYPE_SIMPLE = 0
TYPE_COMPLEX = 1
MAX_PAYLOAD = 1024  # receiving buffer of the document's own example (1.1.3)

# ------------------------------------------------------------------ the source prefix (1.2 note)
# The first byte of a command code names the sender: 0x21 for a remote control terminal (the
# handheld), 0x31 for an autonomous movement module (e.g. navigation). The document writes every
# code with 0x21; the constants below leave the prefix out and code() puts it in.
PREFIX_REMOTE = 0x21
PREFIX_AUTONOMOUS = 0x31
PREFIXES = (PREFIX_REMOTE, PREFIX_AUTONOMOUS)


def code(base: int, prefix: int = PREFIX_AUTONOMOUS) -> int:
    """Return a command code: the source prefix in the top byte, the command below."""
    return ((int(prefix) & 0xFF) << 24) | (int(base) & 0xFFFFFF)


def check_prefix(prefix) -> int:
    """Return the source prefix when it is one of the document's two; ValueError otherwise."""
    value = int(prefix)
    if value not in PREFIXES:
        raise ValueError('command_prefix must be 0x21 (remote control terminal) or 0x31 '
                         '(autonomous module), not 0x{:X}'.format(value))
    return value


# ------------------------------------------------------------------ commands to the motion host
# Commands go to 192.168.1.103:43893 (1.2).
MOTION_HOST_IP = '192.168.1.103'
MOTION_HOST_PORT = 43893

HEARTBEAT = 0x040001            # 1.2.1 at least 2 Hz, or the motion program drops the link
CONNECT = 0x020001              # 1.2.1 sent once after the heartbeat started
MIN_HEARTBEAT_HZ = 2.0


def heartbeat_rate(hz) -> float:
    """Return a heartbeat rate of at least the document's 2 Hz."""
    return max(MIN_HEARTBEAT_HZ, float(hz))


STAND_SIT = 0x010202            # 1.2.2 alternates sitting and standing
TORQUE_CONTROL = 0x01020A       # 1.2.2 initial standing -> torque-controlled standing
STEPPING = 0x010201             # 1.2.2 alternates torque-controlled standing and stepping

# 1.2.3 Axis commands, [-AXIS_MAX, AXIS_MAX], 50 Hz, invalid after 1 s without one.
AXIS_MAX = 32767
AXIS_DEAD_ZONE = 655            # |value| < 655 counts as 0 (1.2.3.2); the driver sends >= 656
VELOCITY_EPSILON = 1e-3         # m/s, rad/s: below this a velocity is 0 (no dead zone bump)
AXIS_LEFT_X = 0x010131          # standing: roll (+ raises the left shoulder); stepping: + right
AXIS_LEFT_Y = 0x010130          # standing: body height (+ up); stepping: + forward
AXIS_RIGHT_X = 0x010135         # standing: yaw (+ head right); stepping: + turns right
AXIS_RIGHT_Y = 0x010102         # standing: pitch (+ head up)
AXIS_FORWARD = AXIS_LEFT_Y
AXIS_LATERAL = AXIS_LEFT_X
AXIS_TURN = AXIS_RIGHT_X

NON_MANUAL_MODE = 0x010C03      # 1.2.4 velocity commands from the perception side are obeyed
MANUAL_MODE = 0x010C02          # 1.2.4 the handheld drives

BODY_HEIGHT = 0x010406          # 1.2.5
HEIGHT_CRAWL = 0
HEIGHT_NORMAL = 2

GAIT_WALK = 0x010300            # 1.2.6, in the stepping state
GAIT_SLOPE = 0x010402
GAIT_OFF_ROAD = 0x010401
GAIT_STAIR = 0x010405           # elevation map: solid ground, grid ground, stairs without risers
GAIT_STAIR_MULTI_FRAME = 0x01040A  # elevation map: multi-frame mode
GAIT_STAIR_45 = 0x01040B        # elevation map: multi-frame mode
GAIT_L_WALK = 0x010420
GAIT_MOUNTAIN = 0x010421
GAIT_SILENT = 0x010422

SOFT_ESTOP = 0x010C0E           # 1.2.8 lies down at once, joint protection

# 1.2.7 Save data: saves the last 5 minutes and exits the motion program. The document prints
# 0x010C01 without the source byte; the 1.2 note gives every command in 1.2 one (the Lite3
# manual writes the same command 0x21010C01), so it goes out as prefix << 24 | 0x010C01.
# Codes below 0x100 (the old 18) go out as they are.
SAVE_DATA = 0x010C01            # jy_exe v2.2.45 or newer
SAVE_DATA_OLD = 18              # older jy_exe


def save_data_command(command: int, prefix: int = PREFIX_AUTONOMOUS) -> int:
    """Return the save data code to send: with the source prefix, except the old raw codes."""
    command = int(command)
    return command if command < 0x100 else code(command, prefix)


# ------------------------------------------------------------------ reports (1.3)
# Sent to every address in the motion host's network.toml (Application Manual 4.3.2).
RCS_DATA = 0x1008               # RcsData, 200 Hz
MOTION_STATE = 0x1009           # MotionStateData, 200 Hz
SENSOR_DATA = 0x100A            # ControllerSensorData (IMU, joints), 200 Hz
SAFE_DATA = 0x100B              # ControllerSafeData (temperatures, CPU), 1 Hz
BATTERY = 0x21050F0A            # BatterySensorData, 0.5 Hz
BODY_HEIGHT_STATUS = 0x11050F08  # simple: -1 crawl, 0 normal

BASIC_STATES = {
    0: 'sitting',
    1: 'sit_to_stand',
    2: 'initial_standing',
    3: 'torque_standing',
    4: 'stepping',
    5: 'stand_to_sit',
    6: 'emergency_stop_or_fall',
}
STATE_SITTING = 0
STATE_SIT_TO_STAND = 1
STATE_INITIAL_STANDING = 2
STATE_TORQUE_STANDING = 3
STATE_STEPPING = 4
STATE_STAND_TO_SIT = 5
STATE_EMERGENCY = 6

GAIT_STATES = {
    0: 'walk',
    1: 'off_road',
    2: 'slope',
    3: 'run',
    6: 'stair',
    7: 'stair_multi_frame',
    8: 'stair_45',
    32: 'l_walk',
    33: 'mountain',
    34: 'silent',
}
GAIT_STATE_WALK = 0
GAIT_STATE_OFF_ROAD = 1
GAIT_STATE_SLOPE = 2
GAIT_STATE_RUN = 3
GAIT_STATE_STAIR = 6
GAIT_STATE_STAIR_MULTI_FRAME = 7
GAIT_STATE_STAIR_45 = 8
GAIT_STATE_L_WALK = 32
GAIT_STATE_MOUNTAIN = 33
GAIT_STATE_SILENT = 34

# gait command -> the gait_state the robot reports once it runs that gait
GAIT_COMMANDS = {
    GAIT_WALK: GAIT_STATE_WALK,
    GAIT_SLOPE: GAIT_STATE_SLOPE,
    GAIT_OFF_ROAD: GAIT_STATE_OFF_ROAD,
    GAIT_STAIR: GAIT_STATE_STAIR,
    GAIT_STAIR_MULTI_FRAME: GAIT_STATE_STAIR_MULTI_FRAME,
    GAIT_STAIR_45: GAIT_STATE_STAIR_45,
    GAIT_L_WALK: GAIT_STATE_L_WALK,
    GAIT_MOUNTAIN: GAIT_STATE_MOUNTAIN,
    GAIT_SILENT: GAIT_STATE_SILENT,
}
# 1.2.5: no crawl height in the off-road, stair or run gaits
NO_CRAWL_GAITS = {GAIT_STATE_OFF_ROAD, GAIT_STATE_RUN, GAIT_STATE_STAIR,
                  GAIT_STATE_STAIR_MULTI_FRAME, GAIT_STATE_STAIR_45}
# 1.2.6: at crawl height only walk and slope
CRAWL_GAITS = {GAIT_STATE_WALK, GAIT_STATE_SLOPE}
# 1.2.6: no body height or terrain switching in L-Walk, Mountain and Silent
FIXED_HEIGHT_GAITS = {GAIT_STATE_L_WALK, GAIT_STATE_MOUNTAIN, GAIT_STATE_SILENT}

EMERGENCY_SOURCES = {
    0: 'none',
    1: 'tail_button',
    2: 'motion',
    3: 'navigation',
    4: 'autonomous_charging',
    5: 'client',
    6: 'ras',
}
# RcsData.error_state, from the lowest bit
ERROR_BITS = (
    'imu_timeout',
    'heartbeat_timeout',
    'driver_overtemperature',
    'driver_failure',
    'motor_overtemperature',
    'low_battery',
    'battery_overheating',
    'gpio_error',
    'cpu_overheating',
    'cpu_frequency_low',
)
POSITION_CONTROL_STATES = {
    0: 'off',
    1: 'initializing',
    2: 'executing',
    3: 'completed',
    4: 'aborted',
}
BODY_HEIGHTS = {-1: 'crawl', 0: 'normal'}

JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in ('FL', 'FR', 'HL', 'HR') for joint in ('HipX', 'HipY', 'Knee')
)

# ------------------------------------------------------------------ Appendix B: gait speeds


@dataclass(frozen=True)
class GaitLimits:
    """Maximum speeds of a gait at normal body height (Appendix B)."""

    forward: float   # m/s
    backward: float  # m/s
    lateral: float   # m/s
    yaw: float       # rad/s


GAIT_LIMITS = {
    GAIT_STATE_WALK: GaitLimits(1.2, 0.75, 0.15, 0.45),
    GAIT_STATE_SLOPE: GaitLimits(0.7, 0.75, 0.25, 0.5),
    GAIT_STATE_OFF_ROAD: GaitLimits(0.3, 0.4, 0.1, 0.5),
    GAIT_STATE_STAIR: GaitLimits(0.3, 0.3, 0.2, 0.8),
    GAIT_STATE_STAIR_MULTI_FRAME: GaitLimits(0.6, 0.3, 0.2, 0.8),
    GAIT_STATE_STAIR_45: GaitLimits(0.3, 0.2, 0.2, 0.8),
    GAIT_STATE_L_WALK: GaitLimits(1.0, 1.0, 0.5, 1.2),
    GAIT_STATE_MOUNTAIN: GaitLimits(1.0, 1.0, 0.8, 1.2),
}
# Run and Silent are not in Appendix B: the driver does not move the robot in them unless the
# gait_limits parameter gives their speeds (limits_table).
DEFAULT_GAIT_LIMITS = GAIT_LIMITS[GAIT_STATE_WALK]  # the simulated robot's guess for them
CRAWL_FORWARD_FACTOR = 0.5  # walk and slope forward speed at crawl height
GAIT_NAMES = {name: state for state, name in GAIT_STATES.items()}


def gait_limits(gait_state: Optional[int], crawl: bool = False,
                table: Optional[Dict[int, GaitLimits]] = None) -> Optional[GaitLimits]:
    """Return a gait's speed limits (crawl halves walk/slope forward); None when unknown."""
    limits = (GAIT_LIMITS if table is None else table).get(gait_state)
    if limits is not None and crawl and gait_state in CRAWL_GAITS:
        limits = GaitLimits(limits.forward * CRAWL_FORWARD_FACTOR, limits.backward,
                            limits.lateral, limits.yaw)
    return limits


def known_gait(gait_state: Optional[int],
               table: Optional[Dict[int, GaitLimits]] = None) -> bool:
    """Return True when the table (Appendix B by default) gives the gait's speeds."""
    return gait_state in (GAIT_LIMITS if table is None else table)


def parse_gait_limits(value) -> Dict[int, GaitLimits]:
    """
    Read the gait_limits parameter: {gait: [forward, backward, lateral, yaw]}.

    The gait is a name of GAIT_STATES (walk, run, silent, ...) or its number; the four speeds
    are positive, in m/s and rad/s. Raise ValueError for anything else.
    """
    if not value:
        return {}
    if not isinstance(value, dict):
        raise ValueError('gait_limits must be a mapping gait: [forward, backward, lateral, yaw]')
    table = {}
    for key, speeds in value.items():
        gait = GAIT_NAMES.get(key, key)
        try:
            gait = int(gait)
        except (TypeError, ValueError):
            raise ValueError('gait_limits: unknown gait {!r} (one of {})'.format(
                key, ', '.join(sorted(GAIT_NAMES)))) from None
        try:
            numbers = [float(v) for v in speeds]
        except (TypeError, ValueError):
            raise ValueError('gait_limits: {} needs four numbers'.format(key)) from None
        if len(numbers) != 4 or not all(math.isfinite(v) and v > 0.0 for v in numbers):
            raise ValueError('gait_limits: {} needs four positive speeds [forward, backward, '
                             'lateral, yaw], not {!r}'.format(key, speeds))
        table[gait] = GaitLimits(*numbers)
    return table


def limits_table(overrides: Optional[Dict[int, GaitLimits]] = None) -> Dict[int, GaitLimits]:
    """Return Appendix B with the overrides on top (they replace or add gaits)."""
    table = dict(GAIT_LIMITS)
    table.update(overrides or {})
    return table


# ------------------------------------------------------------------ perception host (1.4, 1.5)
PERCEPTION_HOST_IP = '192.168.1.105'
ELEVATION_MAP_PORT = 43899      # 1.5: commands and 10 Hz feedback
CHARGE_PORT = 3333              # 1.4: request / response
LIO_PORT = 60000                # 1.5.2.4: LiDAR-inertial odometry for multi-frame mode

ELEVATION_MODE = 0x3101EE01     # 1.5.2: value 3, 4, 5 (20: multi-frame, see the README)
ELEVATION_MODE_STATE = 0x3100EE01
BRAKE_MODE = 0x3101EE02         # 1.5.3: 1 decelerate, 2 detour
BRAKE_MODE_STATE = 0x3100EE02
VELOCITY_SOURCE = 0x3101EE03    # 1.5.1: 1 joystick (assist), 2 navigation module
VELOCITY_SOURCE_STATE = 0x3100EE03
# 1.5.4 as printed: 1 = 8 cm, 2 = 28 cm. It looks like a feedback code (0x3100...); the
# commands are 0x3101EE0x, so 0x3101EE04 is the likely intended one (obstacle_height_code).
OBSTACLE_HEIGHT = 0x3100EE04
OBSTACLE_HEIGHT_LIKELY = 0x3101EE04
ELEVATION_QUERY = 0x3101EEFF    # Appendix C: any message starts the 10 Hz feedback
LIO = 0x0BAA0001                # 1.5.2.4: 1 on, 0 off; reply 0 succeeded, -1 failed
LIO_ON = 1
LIO_OFF = 0
LIO_RESULTS = {0: 'succeeded', -1: 'failed'}

ELEVATION_MODES = {
    3: 'solid_ground',
    4: 'grid_ground',
    5: 'stairs_without_risers',
    18: 'multi_frame_preparing',
    20: 'multi_frame',
}
ELEVATION_SOLID_GROUND = 3
ELEVATION_GRID_GROUND = 4
ELEVATION_NO_RISERS = 5
ELEVATION_MULTI_FRAME_PREPARING = 18
ELEVATION_MULTI_FRAME = 20
# leaving these needs 1.5.2.5: switch the mode, then turn the LiDAR-inertial odometry off
MULTI_FRAME_MODES = (ELEVATION_MULTI_FRAME_PREPARING, ELEVATION_MULTI_FRAME)
BRAKE_MODES = {1: 'decelerate', 2: 'detour'}
VELOCITY_SOURCES = {1: 'joystick', 2: 'navigation'}
OBSTACLE_HEIGHTS = {1: 0.08, 2: 0.28}

CHARGE = 0x91910250             # 1.4: 1 start, 0 stop, 2 clear
CHARGE_QUERY = 0x91910253       # 1.4: value 0
CHARGE_STOP = 0
CHARGE_START = 1
CHARGE_CLEAR = 2
CHARGE_STATES = {
    0x0000: 'idle',
    0x0001: 'startup',
    0x0002: 'charging',
    0x0003: 'exit',
    0x0004: 'power_failure',
    0x0005: 'charging_with_charger',
    0x1002: 'location_timeout',
    0x1003: 'localization_startup_failure',
    0x1004: 'dock_not_reached',
    0x1005: 'no_location',
    0x1006: 'location_spike',
    0x1007: 'no_current_on_dock',
    0x100A: 'no_current_leaving_dock',
}

# ------------------------------------------------------------------ struct layouts
# The document gives plain C structs. The motion program sends them with the compiler's
# natural alignment; pack(1) builds and a 4-byte `long` (RcsData) are recognised by length.
RCS_LAYOUTS = {
    'natural': struct.Struct('<15sxiiqqqq4f10BxxI'),     # 88: 8-byte long (aarch64, x86-64)
    'pack1': struct.Struct('<15siiqqqq4f10BI'),          # 85
    'ilp32': struct.Struct('<15sxiiiiii4f10BxxI'),       # 72: 4-byte long
    'ilp32_pack1': struct.Struct('<15siiiiii4f10BI'),    # 69
}
MOTION_LAYOUTS = {
    'natural': struct.Struct('<BBxx2f3f3ffII10Bxx'),     # 60
    'pack1': struct.Struct('<BB2f3f3ffII10B'),           # 56
}
SENSOR_LAYOUT = struct.Struct('<i9f12f12f12f')           # 184: ImuSensorData + 3 x LegJointData
SAFE_LAYOUT = struct.Struct('<12f12B2f')                 # 68
BATTERY_LAYOUTS = {
    'natural': struct.Struct('<HhHHHHHHH5Bx4f'),         # 40
    'pack1': struct.Struct('<HhHHHHHHH5B4f'),            # 39
}


def _by_size(layouts: Dict[str, struct.Struct]) -> Dict[int, str]:
    return {layout.size: name for name, layout in layouts.items()}


RCS_SIZES = _by_size(RCS_LAYOUTS)
MOTION_SIZES = _by_size(MOTION_LAYOUTS)
BATTERY_SIZES = _by_size(BATTERY_LAYOUTS)


@dataclass(frozen=True)
class Datagram:
    """One datagram: head and payload."""

    code: int
    kind: int
    value: int        # simple: the command value (uint32); complex: the payload length
    payload: bytes


@dataclass(frozen=True)
class RcsData:
    """0x1008: name, mileage, times, handheld axes, control mode, emergency source, errors."""

    robot_name: str
    current_mileage: int      # cm since boot
    total_mileage: int        # cm
    current_run_time: int     # s since boot
    total_run_time: int       # s
    current_motion_time: int  # s since boot
    total_motion_time: int    # s
    joystick: Tuple[float, float, float, float]  # LX, LY, RX, RY in [-1, 1]
    is_nav_mode: int          # 0 manual, 1 non-manual
    emergency_source: int
    error_state: int
    layout: str = 'natural'

    @property
    def non_manual(self) -> bool:
        """Return True in non-manual mode (velocity from the perception side)."""
        return bool(self.is_nav_mode)

    @property
    def errors(self) -> Tuple[str, ...]:
        """Return the names of the error bits that are set."""
        return tuple(name for bit, name in enumerate(ERROR_BITS) if self.error_state >> bit & 1)


@dataclass(frozen=True)
class MotionState:
    """0x1009: basic state, gait, leg odometry."""

    basic_state: int
    gait_state: int
    leg_odom_pos: Tuple[float, float, float]  # x m, y m, yaw rad (world = pose at start-up)
    leg_odom_vel: Tuple[float, float, float]  # vx m/s, vy m/s, yaw rate rad/s (body frame)
    robot_distance: float = 0.0               # mileage since boot, cm
    touch_state: int = 0
    control_state: int = 0
    auto_charge_state: int = 0                # deprecated
    pos_ctrl_state: int = 0
    max_forward_vel: float = 0.0              # deprecated
    max_backward_vel: float = 0.0             # deprecated
    layout: str = 'natural'

    @property
    def stepping(self) -> bool:
        """Return True in the stepping state (the only one that walks)."""
        return self.basic_state == STATE_STEPPING


@dataclass(frozen=True)
class SensorData:
    """0x100A: IMU and the joints' positions, velocities and torques."""

    timestamp: int
    rpy_deg: Tuple[float, float, float]
    omega: Tuple[float, float, float]         # rad/s
    acc: Tuple[float, float, float]           # m/s^2
    joint_pos: Tuple[float, ...]              # rad, JOINT_NAMES order
    joint_vel: Tuple[float, ...]              # rad/s
    joint_tau: Tuple[float, ...]              # N m


@dataclass(frozen=True)
class SafeData:
    """0x100B: motor and driver temperatures, motion host CPU."""

    motor_temperature: Tuple[float, ...]      # deg C, JOINT_NAMES order
    driver_temperature: Tuple[int, ...]       # deg C
    cpu_temperature: float                    # deg C
    cpu_frequency: float                      # MHz


@dataclass(frozen=True)
class BatteryData:
    """0x21050F0A: raw battery fields (units in the README: they depend on the firmware)."""

    voltage: int              # V (1.3.5); the README covers other units
    current: int              # A before Deeprcs 1.10.19, 10 mA from 1.10.19 (1.3.5)
    remaining_capacity: int   # 10 mAh (2.2.1)
    nominal_capacity: int     # 10 mAh (2.2.1)
    cycles: int
    production_date: int
    balanced_low: int
    balanced_high: int
    protected_state: int
    software_version: int
    battery_level: int        # %
    mos_state: int
    battery_quantity: int
    battery_ntc: int
    temperatures: Tuple[float, float, float, float]
    layout: str = 'natural'


# ------------------------------------------------------------------ encoding


def u32(value: int) -> int:
    """Return a signed command value as the uint32 the head carries (two's complement)."""
    return int(value) & 0xFFFFFFFF


def signed(value: int) -> int:
    """Return the uint32 value of a simple command as a signed int."""
    return value - (1 << 32) if value & 0x80000000 else value


def simple(command: int, value: int = 0) -> bytes:
    """Encode a simple command (a full code, prefix included)."""
    return HEAD.pack(u32(command), u32(value), TYPE_SIMPLE)


def complex_command(command: int, payload: bytes) -> bytes:
    """Encode a complex command with its payload."""
    return HEAD.pack(u32(command), len(payload), TYPE_COMPLEX) + payload


def clamp_axis(value) -> int:
    """Clamp an axis value to [-AXIS_MAX, AXIS_MAX]; a non-finite value becomes 0."""
    if isinstance(value, float) and not math.isfinite(value):
        return 0
    return max(-AXIS_MAX, min(AXIS_MAX, int(value)))


def axis(command: int, value) -> bytes:
    """Encode an axis command (a full code), clamped."""
    return simple(command, clamp_axis(value))


def axis_value(velocity: float, maximum: float, dead_zone: int = AXIS_DEAD_ZONE) -> int:
    """
    Return the axis value of a velocity at a gait maximum (1.2.3.2).

    v = value / 32767 * vmax, so value = v / vmax * 32767, clamped. A velocity below
    VELOCITY_EPSILON is 0 (noise must not creep); a larger one that falls inside the dead zone
    (|value| <= 655) is sent as +-656, just outside it, so small commands still move the robot.
    """
    if not math.isfinite(velocity) or maximum <= 0.0 or abs(velocity) < VELOCITY_EPSILON:
        return 0
    value = int(round(velocity / maximum * AXIS_MAX))
    value = max(-AXIS_MAX, min(AXIS_MAX, value))
    if dead_zone > 0 and abs(value) <= dead_zone:
        value = dead_zone + 1 if velocity > 0 else -(dead_zone + 1)
    return value


def velocity_to_axes(vx: float, vy: float, wz: float, limits: GaitLimits,
                     dead_zone: int = AXIS_DEAD_ZONE) -> Tuple[int, int, int]:
    """
    Return (forward, lateral, turn) axis values for a body velocity in REP-103.

    REP-103: x forward, y left, z counter-clockwise. The robot: left Y + forward, left X +
    right, right X + turns right (1.2.3.2), so lateral and turn change sign.
    """
    forward = axis_value(vx, limits.forward if vx >= 0.0 else limits.backward, dead_zone)
    lateral = axis_value(-vy, limits.lateral, dead_zone)
    turn = axis_value(-wz, limits.yaw, dead_zone)
    return forward, lateral, turn


def axes_to_velocity(forward: int, lateral: int, turn: int, limits: GaitLimits,
                     dead_zone: int = AXIS_DEAD_ZONE) -> Tuple[float, float, float]:
    """Return the REP-103 body velocity the robot runs for axis values (1.2.3.2 formulas)."""

    def scale(value: int, maximum: float) -> float:
        value = clamp_axis(value)
        return 0.0 if abs(value) < dead_zone else value / AXIS_MAX * maximum

    vx = scale(forward, limits.forward if forward >= 0 else limits.backward)
    vy = -scale(lateral, limits.lateral)
    wz = -scale(turn, limits.yaw)
    return vx, vy, wz


# ------------------------------------------------------------------ decoding


def parse_datagram(data: bytes) -> Optional[Datagram]:
    """Split a datagram into head and payload; None when it is malformed."""
    if len(data) < HEAD_SIZE:
        return None
    command, size, kind = HEAD.unpack_from(data)
    if kind == TYPE_SIMPLE:
        return Datagram(command, kind, size, b'')
    if kind != TYPE_COMPLEX or size > MAX_PAYLOAD or len(data) < HEAD_SIZE + size:
        return None
    return Datagram(command, kind, size, bytes(data[HEAD_SIZE:HEAD_SIZE + size]))


def name_of(table: dict, value: Optional[int]) -> str:
    """Return the readable name of a code, 'unknown' when it is not in the table."""
    return table.get(value, 'unknown') if value is not None else 'unknown'


def decode_rcs(payload: bytes) -> Optional[RcsData]:
    """Decode RcsData by its length; None for an unknown length."""
    layout = RCS_SIZES.get(len(payload))
    if layout is None:
        return None
    fields = RCS_LAYOUTS[layout].unpack(payload)
    name = fields[0].split(b'\x00', 1)[0].decode('utf-8', 'replace').strip()
    rcs_state = fields[11:21]
    return RcsData(
        robot_name=name,
        current_mileage=fields[1],
        total_mileage=fields[2],
        current_run_time=fields[3],
        total_run_time=fields[4],
        current_motion_time=fields[5],
        total_motion_time=fields[6],
        joystick=tuple(fields[7:11]),
        is_nav_mode=rcs_state[0],
        emergency_source=rcs_state[1],
        error_state=fields[21],
        layout=layout,
    )


def decode_motion_state(payload: bytes) -> Optional[MotionState]:
    """Decode MotionStateData by its length; None for an unknown length."""
    layout = MOTION_SIZES.get(len(payload))
    if layout is None:
        return None
    f = MOTION_LAYOUTS[layout].unpack(payload)
    task_state = f[13:23]
    return MotionState(
        basic_state=f[0],
        gait_state=f[1],
        max_forward_vel=f[2],
        max_backward_vel=f[3],
        leg_odom_pos=tuple(f[4:7]),
        leg_odom_vel=tuple(f[7:10]),
        robot_distance=f[10],
        touch_state=f[11],
        control_state=f[12],
        auto_charge_state=task_state[0],
        pos_ctrl_state=task_state[1],
        layout=layout,
    )


def decode_sensor(payload: bytes) -> Optional[SensorData]:
    """Decode ControllerSensorData; None when the payload is too short."""
    if len(payload) < SENSOR_LAYOUT.size:
        return None
    f = SENSOR_LAYOUT.unpack_from(payload)
    return SensorData(
        timestamp=f[0],
        rpy_deg=tuple(f[1:4]),
        omega=tuple(f[4:7]),
        acc=tuple(f[7:10]),
        joint_pos=tuple(f[10:22]),
        joint_vel=tuple(f[22:34]),
        joint_tau=tuple(f[34:46]),
    )


def decode_safe(payload: bytes) -> Optional[SafeData]:
    """Decode ControllerSafeData; None when the payload is too short."""
    if len(payload) < SAFE_LAYOUT.size:
        return None
    f = SAFE_LAYOUT.unpack_from(payload)
    return SafeData(
        motor_temperature=tuple(f[0:12]),
        driver_temperature=tuple(f[12:24]),
        cpu_temperature=f[24],
        cpu_frequency=f[25],
    )


def decode_battery(payload: bytes) -> Optional[BatteryData]:
    """Decode BatterySensorData by its length; None for an unknown length."""
    layout = BATTERY_SIZES.get(len(payload))
    if layout is None:
        return None
    f = BATTERY_LAYOUTS[layout].unpack(payload)
    return BatteryData(*f[:14], temperatures=tuple(f[14:18]), layout=layout)


# ------------------------------------------------------------------ encoding of reports
# (the simulated robot and the tests)


def encode_rcs(data: RcsData, layout: str = 'natural') -> bytes:
    """Encode RcsData as the motion host would."""
    rcs_state = [data.is_nav_mode, data.emergency_source] + [0] * 8
    payload = RCS_LAYOUTS[layout].pack(
        data.robot_name.encode('utf-8')[:15], data.current_mileage, data.total_mileage,
        data.current_run_time, data.total_run_time, data.current_motion_time,
        data.total_motion_time, *data.joystick, *rcs_state, data.error_state,
    )
    return complex_command(RCS_DATA, payload)


def encode_motion_state(data: MotionState, layout: str = 'natural') -> bytes:
    """Encode MotionStateData as the motion host would."""
    task_state = [data.auto_charge_state, data.pos_ctrl_state] + [0] * 8
    payload = MOTION_LAYOUTS[layout].pack(
        data.basic_state, data.gait_state, data.max_forward_vel, data.max_backward_vel,
        *data.leg_odom_pos, *data.leg_odom_vel, data.robot_distance, data.touch_state,
        data.control_state, *task_state,
    )
    return complex_command(MOTION_STATE, payload)


def encode_sensor(data: SensorData) -> bytes:
    """Encode ControllerSensorData as the motion host would."""
    payload = SENSOR_LAYOUT.pack(data.timestamp, *data.rpy_deg, *data.omega, *data.acc,
                                 *data.joint_pos, *data.joint_vel, *data.joint_tau)
    return complex_command(SENSOR_DATA, payload)


def encode_safe(data: SafeData) -> bytes:
    """Encode ControllerSafeData as the motion host would."""
    payload = SAFE_LAYOUT.pack(*data.motor_temperature, *data.driver_temperature,
                               data.cpu_temperature, data.cpu_frequency)
    return complex_command(SAFE_DATA, payload)


def encode_battery(data: BatteryData, layout: str = 'natural') -> bytes:
    """Encode BatterySensorData as the motion host would."""
    payload = BATTERY_LAYOUTS[layout].pack(
        data.voltage, data.current, data.remaining_capacity, data.nominal_capacity, data.cycles,
        data.production_date, data.balanced_low, data.balanced_high, data.protected_state,
        data.software_version, data.battery_level, data.mos_state, data.battery_quantity,
        data.battery_ntc, *data.temperatures,
    )
    return complex_command(BATTERY, payload)
