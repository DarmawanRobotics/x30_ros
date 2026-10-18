"""
ROS 1 (Noetic) driver of the Deep Robotics Jueying X30 over the motion host's UDP protocol.

cmd_vel becomes axis commands in the stepping state (scaled by the current gait's speeds); the
motion host's reports become odometry, IMU, joint states, battery and a state message. Every
robot command is a std_srvs/Trigger service; the perception host's elevation map and
autonomous charging have services too.
"""

import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_srvs.srv import Trigger, TriggerResponse
from x30_driver import protocol as p
from x30_driver.convert import (
    attitude,
    battery_fraction,
    battery_voltage,
    max_finite,
    odometry_orientation,
    quaternion_from_rpy,
)
from x30_driver.link import UdpLink
from x30_driver.motion import AutoConfig, AutoSwitch, VelocityConfig, VelocityPolicy

GAIT_SERVICES = {
    'gait_walk': p.GAIT_WALK,
    'gait_slope': p.GAIT_SLOPE,
    'gait_off_road': p.GAIT_OFF_ROAD,
    'gait_stair': p.GAIT_STAIR,
    'gait_stair_multi_frame': p.GAIT_STAIR_MULTI_FRAME,
    'gait_stair_45': p.GAIT_STAIR_45,
    'gait_l_walk': p.GAIT_L_WALK,
    'gait_mountain': p.GAIT_MOUNTAIN,
    'gait_silent': p.GAIT_SILENT,
}
# stair gaits and the elevation map modes they need (1.2.6 note)
STAIR_ELEVATION = {
    p.GAIT_STAIR: (p.ELEVATION_SOLID_GROUND, p.ELEVATION_GRID_GROUND, p.ELEVATION_NO_RISERS),
    p.GAIT_STAIR_MULTI_FRAME: (p.ELEVATION_MULTI_FRAME,),
    p.GAIT_STAIR_45: (p.ELEVATION_MULTI_FRAME,),
}
# service -> (code, value, port parameter, feedback attribute, expected feedback value)
PERCEPTION_SERVICES = {
    'elevation_solid_ground': (p.ELEVATION_MODE, p.ELEVATION_SOLID_GROUND, 'vmap',
                               'elevation_mode', p.ELEVATION_SOLID_GROUND),
    'elevation_grid_ground': (p.ELEVATION_MODE, p.ELEVATION_GRID_GROUND, 'vmap',
                              'elevation_mode', p.ELEVATION_GRID_GROUND),
    'elevation_stairs_no_risers': (p.ELEVATION_MODE, p.ELEVATION_NO_RISERS, 'vmap',
                                   'elevation_mode', p.ELEVATION_NO_RISERS),
    'obstacle_decelerate': (p.BRAKE_MODE, 1, 'vmap', 'brake_mode', 1),
    'obstacle_detour': (p.BRAKE_MODE, 2, 'vmap', 'brake_mode', 2),
    'velocity_source_joystick': (p.VELOCITY_SOURCE, 1, 'vmap', 'velocity_source', 1),
    'velocity_source_navigation': (p.VELOCITY_SOURCE, 2, 'vmap', 'velocity_source', 2),
    'obstacle_height_8cm': (p.OBSTACLE_HEIGHT, 1, 'vmap', None, None),
    'obstacle_height_28cm': (p.OBSTACLE_HEIGHT, 2, 'vmap', None, None),
    'charge_start': (p.CHARGE, p.CHARGE_START, 'charge', 'charge_state', None),
    'charge_stop': (p.CHARGE, p.CHARGE_STOP, 'charge', 'charge_state', None),
    'charge_clear': (p.CHARGE, p.CHARGE_CLEAR, 'charge', 'charge_state', None),
}
STANDING = (p.STATE_INITIAL_STANDING, p.STATE_TORQUE_STANDING, p.STATE_STEPPING)
TRANSITIONS = (p.STATE_SIT_TO_STAND, p.STATE_STAND_TO_SIT)


def param(name, default):
    """Return a private parameter."""
    return rospy.get_param('~' + name, default)


class X30Driver:
    """Bridges ROS and the X30 motion and perception hosts."""

    def __init__(self):
        """Read parameters, open the UDP links and create every interface."""
        self.prefix = int(param('command_prefix', p.PREFIX_AUTONOMOUS))
        self.save_data_code = int(param('save_data_code', p.SAVE_DATA))
        self.velocity_mode = str(param('velocity_mode', 'axis'))
        if self.velocity_mode not in ('axis', 'topic'):
            rospy.logwarn("velocity_mode '%s' unknown, using 'axis'", self.velocity_mode)
            self.velocity_mode = 'axis'
        self.odom_frame = param('odom_frame', 'odom')
        self.base_frame = param('base_frame', 'base_link')
        self.imu_frame = param('imu_frame', 'imu_link')
        self.publish_tf = bool(param('publish_tf', False))
        self.state_timeout = float(param('state_timeout_sec', 1.0))
        self.low_battery = float(param('low_battery', 0.2))
        self.battery_voltage_scale = float(param('battery_voltage_scale', 0.0))
        self.battery_current_scale = float(param('battery_current_scale', 0.01))
        self.connect_retry = float(param('connect_retry_sec', 5.0))
        self.stand_timeout = float(param('stand_timeout_sec', 10.0))
        self.transition_timeout = float(param('transition_timeout_sec', 5.0))
        self.feedback_timeout = float(param('feedback_timeout_sec', 2.0))
        self.auto_torque_control = bool(param('auto_torque_control', True))

        self.policy = VelocityPolicy(VelocityConfig(
            max_vx=float(param('max_vx', 1.0)),
            max_vy=float(param('max_vy', 0.5)),
            max_wz=float(param('max_wz', 1.0)),
            cmd_timeout_sec=float(param('cmd_timeout_sec', 0.5)),
            stop_burst_sec=float(param('stop_burst_sec', 0.5)),
            invert_vy=bool(param('invert_vy', False)),
            invert_wz=bool(param('invert_wz', False)),
            dead_zone=int(param('axis_dead_zone', p.AXIS_DEAD_ZONE)),
        ))
        self.auto = AutoSwitch(AutoConfig(
            auto_non_manual_mode=bool(param('auto_non_manual_mode', True)),
            auto_stepping=bool(param('auto_stepping', True)),
            mode_refresh_sec=float(param('mode_refresh_sec', 2.0)),
            stepping_retry_sec=float(param('stepping_retry_sec', 2.0)),
            idle_stop_stepping_sec=float(param('idle_stop_stepping_sec', 0.0)),
        ))
        self._policy_lock = threading.Lock()

        self._lock = threading.Lock()
        self._rcs = None
        self._motion = None
        self._sensor = None
        self._safe = None
        self._battery = None
        self._height = None
        self._last_report = None
        self._new_motion = False
        self._new_sensor = False
        self._new_battery = False
        self._warned = set()
        self._unknown_codes = {}
        self._ignored = ''
        self._connect_time = None
        self._was_connected = None
        self._feedback = {'elevation_mode': None, 'brake_mode': None, 'velocity_source': None,
                          'charge_state': None}
        self._feedback_time = {}
        self._elevation_query_time = None
        self._charge_query_time = None

        self.link = UdpLink(
            param('motion_host_ip', p.MOTION_HOST_IP),
            int(param('motion_host_port', p.MOTION_HOST_PORT)),
            param('local_ip', '0.0.0.0'),
            int(param('local_port', 43897)),
            self.on_datagram,
            name='x30_motion',
        )
        perception_ip = str(param('perception_host_ip', p.PERCEPTION_HOST_IP) or '')
        self.ports = {
            'vmap': int(param('elevation_map_port', p.ELEVATION_MAP_PORT)),
            'charge': int(param('charge_port', p.CHARGE_PORT)),
        }
        self.perception = None
        if perception_ip:
            self.perception = UdpLink(perception_ip, self.ports['vmap'], '0.0.0.0', 0,
                                      self.on_perception, name='x30_perception')
        self.elevation_query_sec = float(param('elevation_query_sec', 5.0))
        self.charge_query_sec = float(param('charge_query_sec', 2.0))

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.battery_pub = rospy.Publisher('battery', BatteryState, queue_size=10)
        self.state_pub = rospy.Publisher('state', DiagnosticStatus, queue_size=10)
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)
        self.velocity_pub = None
        if self.velocity_mode == 'topic':
            self.velocity_pub = rospy.Publisher(
                str(param('velocity_topic', '/cmd_vel_corrected')), Twist, queue_size=10)
        self.tf_broadcaster = None
        if self.publish_tf:
            import tf2_ros  # only needed with publish_tf
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('cmd_vel', Twist, self.on_cmd_vel, queue_size=10)

        handlers = {
            'stand_up': self.on_stand_up,
            'sit_down': self.on_sit_down,
            'torque_control': self.on_torque_control,
            'start_stepping': self.on_start_stepping,
            'stop_stepping': self.on_stop_stepping,
            'non_manual_mode': self.mode_callback(p.NON_MANUAL_MODE, True),
            'manual_mode': self.mode_callback(p.MANUAL_MODE, False),
            'height_normal': self.height_callback(p.HEIGHT_NORMAL),
            'height_crawl': self.height_callback(p.HEIGHT_CRAWL),
            'emergency_stop': self.on_emergency_stop,
            'save_data': self.on_save_data,
        }
        for name, base in GAIT_SERVICES.items():
            handlers[name] = self.gait_callback(name, base)
        for name, spec in PERCEPTION_SERVICES.items():
            handlers[name] = self.perception_callback(name, *spec)
        self._services = [rospy.Service(name, Trigger, handler)
                          for name, handler in handlers.items()]

        self.link.start()
        if self.perception is not None:
            self.perception.start()
        self.send_heartbeat()  # heartbeat and connect right away, then on the timer

        def rate(name, default):
            return rospy.Duration(1.0 / max(float(param(name, default)), 0.01))

        self._timers = [
            rospy.Timer(rate('heartbeat_hz', 5.0), self.send_heartbeat),
            rospy.Timer(rate('cmd_rate_hz', 50.0), self.send_velocity),
            rospy.Timer(rate('odom_hz', 50.0), self.publish_odometry),
            rospy.Timer(rate('imu_hz', 100.0), self.publish_imu),
            rospy.Timer(rate('joint_state_hz', 50.0), self.publish_joints),
            rospy.Timer(rate('state_hz', 10.0), self.publish_state),
            rospy.Timer(rate('battery_hz', 1.0), self.publish_battery),
            rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics),
        ]
        if self.perception is not None:
            self._timers.append(rospy.Timer(rospy.Duration(0.5), self.query_perception))
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            'X30 motion host %s:%d (prefix 0x%02X), reports on %s:%d (listed in network.toml on '
            'the motion host?), perception host %s, velocity mode %s',
            self.link.remote[0], self.link.remote[1], self.prefix, self.link.local[0],
            self.link.local[1], perception_ip or 'off', self.velocity_mode)

    # ------------------------------------------------------------------ link

    def warn_once(self, key, message, *args):
        """Log a warning once per key."""
        if key not in self._warned:
            self._warned.add(key)
            rospy.logwarn(message, *args)

    def on_datagram(self, data, _address):
        """Store one report from the motion host (receive thread)."""
        datagram = p.parse_datagram(data)
        if datagram is None:
            return
        now = time.monotonic()
        c = datagram.code
        if datagram.kind == p.TYPE_SIMPLE:
            if c == p.BODY_HEIGHT_STATUS:
                with self._lock:
                    self._height = p.signed(datagram.value)
                    self._last_report = now
            else:
                self._unknown_codes[c] = self._unknown_codes.get(c, 0) + 1
            return
        decoders = {
            p.RCS_DATA: ('_rcs', p.decode_rcs, None),
            p.MOTION_STATE: ('_motion', p.decode_motion_state, '_new_motion'),
            p.SENSOR_DATA: ('_sensor', p.decode_sensor, '_new_sensor'),
            p.SAFE_DATA: ('_safe', p.decode_safe, None),
            p.BATTERY: ('_battery', p.decode_battery, '_new_battery'),
        }
        if c not in decoders:
            self._unknown_codes[c] = self._unknown_codes.get(c, 0) + 1
            return
        attr, decode, flag = decoders[c]
        value = decode(datagram.payload)
        if value is None:
            self.warn_once(('layout', c), 'report 0x%X of %d bytes: unknown layout, skipped',
                           c, len(datagram.payload))
            return
        with self._lock:
            setattr(self, attr, value)
            if flag:
                setattr(self, flag, True)
            self._last_report = now

    def on_perception(self, data, _address):
        """Store a reply of the perception host (elevation map feedback, charging state)."""
        datagram = p.parse_datagram(data)
        if datagram is None or datagram.kind != p.TYPE_SIMPLE:
            return
        keys = {
            p.ELEVATION_MODE_STATE: 'elevation_mode',
            p.BRAKE_MODE_STATE: 'brake_mode',
            p.VELOCITY_SOURCE_STATE: 'velocity_source',
            p.CHARGE: 'charge_state',
            p.CHARGE_QUERY: 'charge_state',
        }
        key = keys.get(datagram.code)
        if key is None:
            return
        now = time.monotonic()
        with self._lock:
            self._feedback[key] = p.signed(datagram.value)
            self._feedback_time[key] = now
            if datagram.code == p.CHARGE:  # the answer to a charging request, not to a query
                self._feedback_time['charge_reply'] = now

    def snapshot(self):
        """Return (rcs, motion, sensor, safe, battery, height) under the lock."""
        with self._lock:
            return (self._rcs, self._motion, self._sensor, self._safe, self._battery,
                    self._height)

    def motion(self):
        """Return the latest MotionStateData, or None."""
        with self._lock:
            return self._motion

    def age(self) -> float:
        """Return seconds since the last report."""
        with self._lock:
            stamp = self._last_report
        return float('inf') if stamp is None else time.monotonic() - stamp

    def connected(self) -> bool:
        """Return True while reports keep arriving."""
        return self.age() <= self.state_timeout

    def send_base(self, base, value=0) -> bool:
        """Send a simple command with the configured source prefix."""
        return self.link.send(p.simple(p.code(base, self.prefix), value))

    def send_axes(self, axes) -> bool:
        """Send the forward, lateral and turn axis commands."""
        forward, lateral, turn = axes
        return self.link.send_all((
            p.axis(p.code(p.AXIS_FORWARD, self.prefix), forward),
            p.axis(p.code(p.AXIS_LATERAL, self.prefix), lateral),
            p.axis(p.code(p.AXIS_TURN, self.prefix), turn),
        )) == 3

    # ------------------------------------------------------------------ motion

    def on_cmd_vel(self, msg):
        """Take a velocity command (base_link, REP-103)."""
        with self._policy_lock:
            self.policy.command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def send_heartbeat(self, _event=None):
        """
        Send the heartbeat (1.2.1), and the connect command when it is due.

        Connect goes out once at start, every connect_retry_sec while no reports come, and once
        more when they come back.
        """
        self.send_base(p.HEARTBEAT)
        now = time.monotonic()
        connected = self.connected()
        returned = connected and self._was_connected is False
        if self._connect_time is None or returned or (
                not connected and now - self._connect_time >= self.connect_retry):
            self.send_base(p.CONNECT)
            self._connect_time = now
        self._was_connected = connected

    def send_velocity(self, _event=None):
        """Stream the velocity: mode and stepping switches, then the axis commands."""
        now = time.monotonic()
        rcs, motion, _, _, _, height = self.snapshot()
        connected = self.connected()
        basic = motion.basic_state if motion is not None and connected else None
        non_manual = rcs.non_manual if rcs is not None and connected else None
        with self._policy_lock:
            moving = self.policy.moving(now)
            cmd = self.policy.tick(now)
        for base in self.auto.decide(now, moving, basic, non_manual):
            self.send_base(base)
        if cmd is None:
            self._ignored = ''
            return
        if self.velocity_mode == 'topic':
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            self.velocity_pub.publish(msg)
            return
        if basic != p.STATE_STEPPING:
            if moving:
                self._ignored = ('no reports from the motion host' if basic is None else
                                 'not stepping ({})'.format(p.name_of(p.BASIC_STATES, basic)))
            return
        self._ignored = 'manual mode: the handheld drives' if (
            moving and non_manual is False) else ''
        with self._policy_lock:
            axes = self.policy.axes(cmd, motion.gait_state, height == -1)
        self.send_axes(axes)

    def zero_velocity(self):
        """Drop the command and, while stepping, send zero axes once."""
        with self._policy_lock:
            self.policy.stop(time.monotonic())
        motion = self.motion()
        if motion is not None and motion.stepping and self.velocity_mode == 'axis':
            self.send_axes((0, 0, 0))

    # ------------------------------------------------------------------ services

    def wait_for(self, predicate, timeout):
        """Wait until predicate(motion) holds; return the last MotionStateData, None on timeout."""
        end = time.monotonic() + timeout
        while not rospy.is_shutdown():
            motion = self.motion()
            if motion is not None and predicate(motion):
                return motion
            if time.monotonic() >= end:
                return None
            time.sleep(0.05)
        return None

    def basic_state(self):
        """Return the current basic state, None when there are no reports."""
        motion = self.motion()
        return motion.basic_state if motion is not None and self.connected() else None

    @staticmethod
    def reply(success, message):
        """Return a Trigger response."""
        return TriggerResponse(success=bool(success), message=message)

    def no_reports(self):
        """Return the response for commands that need the robot's state."""
        return self.reply(False, 'no reports from the motion host (is this computer in its '
                                 'network.toml?)')

    def state_name(self, state):
        """Return a basic state's name."""
        return p.name_of(p.BASIC_STATES, state)

    def to_torque_standing(self):
        """From initial standing: torque control, waiting for it. Return (success, message)."""
        self.send_base(p.TORQUE_CONTROL)
        if self.wait_for(lambda m: m.basic_state in (p.STATE_TORQUE_STANDING, p.STATE_STEPPING),
                         self.transition_timeout) is None:
            return False, 'torque control sent, still {}'.format(
                self.state_name(self.basic_state()))
        return True, 'standing (torque control)'

    def on_stand_up(self, _request):
        """Stand up from sitting, then (auto_torque_control) torque-controlled standing."""
        state = self.basic_state()
        if state is None:
            return self.no_reports()
        if state == p.STATE_INITIAL_STANDING and self.auto_torque_control:
            return self.reply(*self.to_torque_standing())
        if state in STANDING:
            return self.reply(True, 'already standing ({})'.format(self.state_name(state)))
        if state in TRANSITIONS:
            return self.reply(False, 'busy ({})'.format(self.state_name(state)))
        if state == p.STATE_EMERGENCY:
            return self.reply(False, 'emergency stop or fall: clear it on the robot first')
        self.send_base(p.STAND_SIT)
        if self.wait_for(lambda m: m.basic_state in STANDING, self.stand_timeout) is None:
            return self.reply(False, 'stand/sit sent, still {}'.format(
                self.state_name(self.basic_state())))
        if self.auto_torque_control and self.basic_state() == p.STATE_INITIAL_STANDING:
            return self.reply(*self.to_torque_standing())
        return self.reply(True, 'standing ({})'.format(self.state_name(self.basic_state())))

    def on_sit_down(self, _request):
        """Sit down from standing; from stepping in place it stops stepping first."""
        state = self.basic_state()
        if state is None:
            return self.no_reports()
        if state == p.STATE_SITTING:
            return self.reply(True, 'already sitting')
        if state == p.STATE_STEPPING:
            with self._policy_lock:
                walking = self.policy.recently_moved(time.monotonic(), 1.0)
            if walking:
                return self.reply(False, 'the robot is walking')
            self.zero_velocity()
            self.send_base(p.STEPPING)
            if self.wait_for(lambda m: m.basic_state == p.STATE_TORQUE_STANDING,
                             self.transition_timeout) is None:
                return self.reply(False, 'stop stepping sent, still {}'.format(
                    self.state_name(self.basic_state())))
            state = p.STATE_TORQUE_STANDING
        if state in (p.STATE_INITIAL_STANDING, p.STATE_TORQUE_STANDING):
            self.send_base(p.STAND_SIT)
            if self.wait_for(lambda m: m.basic_state == p.STATE_SITTING,
                             self.stand_timeout) is None:
                return self.reply(False, 'stand/sit sent, still {}'.format(
                    self.state_name(self.basic_state())))
            return self.reply(True, 'sitting')
        if state == p.STATE_EMERGENCY:
            return self.reply(False, 'emergency stop or fall: clear it on the robot first')
        return self.reply(False, 'busy ({})'.format(self.state_name(state)))

    def on_torque_control(self, _request):
        """Go from initial standing to torque-controlled standing."""
        state = self.basic_state()
        if state is None:
            return self.no_reports()
        if state in (p.STATE_TORQUE_STANDING, p.STATE_STEPPING):
            return self.reply(True, 'already in torque control ({})'.format(
                self.state_name(state)))
        if state != p.STATE_INITIAL_STANDING:
            return self.reply(False, 'stand up first ({})'.format(self.state_name(state)))
        return self.reply(*self.to_torque_standing())

    def on_start_stepping(self, _request):
        """Torque-controlled standing -> stepping (from initial standing via torque control)."""
        state = self.basic_state()
        if state is None:
            return self.no_reports()
        if state == p.STATE_STEPPING:
            return self.reply(True, 'already stepping')
        if state == p.STATE_INITIAL_STANDING:
            ok, message = self.to_torque_standing()
            if not ok:
                return self.reply(False, message)
            state = self.basic_state()
        if state != p.STATE_TORQUE_STANDING:
            return self.reply(False, 'stand up first ({})'.format(self.state_name(state)))
        self.send_base(p.STEPPING)
        if self.wait_for(lambda m: m.basic_state == p.STATE_STEPPING,
                         self.transition_timeout) is None:
            return self.reply(False, 'start stepping sent, still {}'.format(
                self.state_name(self.basic_state())))
        return self.reply(True, 'stepping')

    def on_stop_stepping(self, _request):
        """Go from stepping to torque-controlled standing, after zeroing the velocity."""
        state = self.basic_state()
        if state is None:
            return self.no_reports()
        if state in (p.STATE_TORQUE_STANDING, p.STATE_INITIAL_STANDING):
            return self.reply(True, 'already standing ({})'.format(self.state_name(state)))
        if state != p.STATE_STEPPING:
            return self.reply(False, 'not stepping ({})'.format(self.state_name(state)))
        self.zero_velocity()
        self.send_base(p.STEPPING)
        if self.wait_for(lambda m: m.basic_state == p.STATE_TORQUE_STANDING,
                         self.transition_timeout) is None:
            return self.reply(False, 'stop stepping sent, still {}'.format(
                self.state_name(self.basic_state())))
        return self.reply(True, 'standing (torque control)')

    def wait_rcs(self, predicate, timeout):
        """Wait until predicate(rcs) holds; True when it did."""
        end = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < end:
            with self._lock:
                rcs = self._rcs
            if rcs is not None and predicate(rcs):
                return True
            time.sleep(0.05)
        return False

    def mode_callback(self, base, non_manual):
        """Return the service callback of a control mode."""
        name = 'non-manual' if non_manual else 'manual'

        def callback(_request):
            if not self.send_base(base):
                return self.reply(False, self.link.last_error)
            if not self.connected():
                return self.reply(True, '{} mode sent (no reports from the motion host)'.format(
                    name))
            if self.wait_rcs(lambda r: r.non_manual == non_manual, self.feedback_timeout):
                return self.reply(True, '{} mode'.format(name))
            return self.reply(False, '{} mode sent, the robot still reports the other'.format(
                name))
        return callback

    def height_callback(self, value):
        """Return the service callback of a body height."""
        crawl = value == p.HEIGHT_CRAWL
        name = 'crawl' if crawl else 'normal'

        def callback(_request):
            motion = self.motion()
            if motion is not None and self.connected():
                gait = motion.gait_state
                if gait in p.FIXED_HEIGHT_GAITS:
                    return self.reply(False, 'no height switching in the {} gait'.format(
                        p.name_of(p.GAIT_STATES, gait)))
                if crawl and gait in p.NO_CRAWL_GAITS:
                    return self.reply(False, 'no crawl height in the {} gait'.format(
                        p.name_of(p.GAIT_STATES, gait)))
            if not self.send_base(p.BODY_HEIGHT, value):
                return self.reply(False, self.link.last_error)
            target = -1 if crawl else 0
            end = time.monotonic() + self.feedback_timeout
            while time.monotonic() < end and not rospy.is_shutdown():
                with self._lock:
                    height = self._height
                if height == target:
                    return self.reply(True, '{} height'.format(name))
                time.sleep(0.05)
            return self.reply(True, '{} height sent (no body height report yet)'.format(name))
        return callback

    def gait_callback(self, name, base):
        """Return the service callback of a gait (stepping state only)."""
        expected = p.GAIT_COMMANDS[base]

        def callback(_request):
            motion = self.motion()
            if motion is None or not self.connected():
                return self.no_reports()
            if motion.basic_state != p.STATE_STEPPING:
                return self.reply(False, 'gaits change in the stepping state ({}): call '
                                         'start_stepping'.format(
                                             self.state_name(motion.basic_state)))
            with self._lock:
                height = self._height
                elevation = self._feedback['elevation_mode']
            if height == -1 and expected not in p.CRAWL_GAITS:
                return self.reply(False, 'only walk and slope at crawl height')
            needed = STAIR_ELEVATION.get(base)
            if needed is not None and elevation is not None and elevation not in needed:
                return self.reply(False, '{} needs the elevation map in {} (now {})'.format(
                    name, ' / '.join(p.name_of(p.ELEVATION_MODES, m) for m in needed),
                    p.name_of(p.ELEVATION_MODES, elevation)))
            if not self.send_base(base):
                return self.reply(False, self.link.last_error)
            if self.wait_for(lambda m: m.gait_state == expected, self.feedback_timeout) is None:
                gait = self.motion().gait_state
                return self.reply(False, '{} sent, the robot still reports {}'.format(
                    name, p.name_of(p.GAIT_STATES, gait)))
            return self.reply(True, p.name_of(p.GAIT_STATES, expected))
        return callback

    def on_emergency_stop(self, _request):
        """Zero the velocity and send the soft emergency stop."""
        self.zero_velocity()
        if not self.send_base(p.SOFT_ESTOP):
            return self.reply(False, self.link.last_error)
        return self.reply(True, 'soft emergency stop sent')

    def on_save_data(self, _request):
        """Save the last 5 minutes of data; the robot then exits its motion program."""
        self.zero_velocity()
        if not self.link.send(p.simple(self.save_data_code)):
            return self.reply(False, self.link.last_error)
        return self.reply(True, 'save data (0x{:X}) sent: the robot saves its data and exits its '
                                'motion program'.format(self.save_data_code))

    def perception_callback(self, name, command, value, port, key, expected):
        """Return the service callback of an elevation map or charging command."""
        def callback(_request):
            if self.perception is None:
                return self.reply(False, 'perception_host_ip is empty: perception host disabled')
            address = (self.perception.remote[0], self.ports[port])
            stamp_key = 'charge_reply' if command == p.CHARGE else key
            with self._lock:
                before = self._feedback_time.get(stamp_key) if key else None
            sent = time.monotonic()
            if not self.perception.send(p.simple(command, value), address):
                return self.reply(False, self.perception.last_error)
            if key is None:
                return self.reply(True, '{} sent'.format(name))
            end = sent + self.feedback_timeout
            while time.monotonic() < end and not rospy.is_shutdown():
                with self._lock:
                    current = self._feedback.get(key)
                    stamp = self._feedback_time.get(stamp_key)
                fresh = stamp is not None and stamp != before and stamp >= sent
                if fresh and (expected is None or current == expected):
                    return self.reply(True, '{}: {}'.format(
                        name, self.feedback_name(key, current)))
                time.sleep(0.05)
            with self._lock:
                current = self._feedback.get(key)
            if current is None:
                return self.reply(True, '{} sent (no answer from the perception host)'.format(
                    name))
            return self.reply(expected is None, '{} sent, the perception host reports {}'.format(
                name, self.feedback_name(key, current)))
        return callback

    @staticmethod
    def feedback_name(key, value):
        """Return the readable name of a perception host value."""
        tables = {
            'elevation_mode': p.ELEVATION_MODES,
            'brake_mode': p.BRAKE_MODES,
            'velocity_source': p.VELOCITY_SOURCES,
            'charge_state': p.CHARGE_STATES,
        }
        return p.name_of(tables[key], value) if value is not None else 'unknown'

    def query_perception(self, _event=None):
        """Keep the elevation map feedback flowing and ask for the charging state."""
        now = time.monotonic()
        address = self.perception.remote[0]
        with self._lock:
            vmap_time = self._feedback_time.get('elevation_mode')
        if (vmap_time is None or now - vmap_time > self.feedback_timeout) and (
                self._elevation_query_time is None
                or now - self._elevation_query_time >= self.elevation_query_sec):
            self.perception.send(p.simple(p.ELEVATION_QUERY, 0), (address, self.ports['vmap']))
            self._elevation_query_time = now
        if self.charge_query_sec > 0.0 and (
                self._charge_query_time is None
                or now - self._charge_query_time >= self.charge_query_sec):
            self.perception.send(p.simple(p.CHARGE_QUERY, 0), (address, self.ports['charge']))
            self._charge_query_time = now

    # ------------------------------------------------------------------ publishing

    def publish_odometry(self, _event=None):
        """Publish the leg odometry for each new motion state report."""
        with self._lock:
            motion, sensor, fresh = self._motion, self._sensor, self._new_motion
            self._new_motion = False
        if motion is None or not fresh:
            return
        stamp = rospy.Time.now()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(motion.leg_odom_pos[0])
        msg.pose.pose.position.y = float(motion.leg_odom_pos[1])
        q = odometry_orientation(motion, sensor)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = float(motion.leg_odom_vel[0])
        msg.twist.twist.linear.y = float(motion.leg_odom_vel[1])
        msg.twist.twist.angular.z = float(motion.leg_odom_vel[2])
        pose_cov = list(msg.pose.covariance)
        twist_cov = list(msg.twist.covariance)
        for i, variance in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            pose_cov[i * 7] = variance
        for i, variance in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.05)):
            twist_cov[i * 7] = variance
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.odom_pub.publish(msg)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, _event=None):
        """Publish the IMU for each new sensor report."""
        with self._lock:
            sensor, fresh = self._sensor, self._new_sensor
            self._new_sensor = False
        if sensor is None or not fresh:
            return
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.imu_frame
        o = msg.orientation
        o.x, o.y, o.z, o.w = quaternion_from_rpy(*attitude(sensor))
        av, la = msg.angular_velocity, msg.linear_acceleration
        av.x, av.y, av.z = (float(v) for v in sensor.omega)
        la.x, la.y, la.z = (float(v) for v in sensor.acc)
        self.imu_pub.publish(msg)

    def publish_joints(self, _event=None):
        """Publish joint positions, velocities and torques in the URDF's joint names."""
        with self._lock:
            sensor = self._sensor
        if sensor is None or not self.connected():
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(p.JOINT_NAMES)
        msg.position = [float(v) for v in sensor.joint_pos]
        msg.velocity = [float(v) for v in sensor.joint_vel]
        msg.effort = [float(v) for v in sensor.joint_tau]
        self.joint_pub.publish(msg)

    def publish_battery(self, _event=None):
        """Publish the battery report."""
        with self._lock:
            battery = self._battery
        if battery is None:
            return
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.percentage = battery_fraction(battery.battery_level)
        msg.voltage = battery_voltage(battery.voltage, self.battery_voltage_scale)
        msg.current = battery.current * self.battery_current_scale
        msg.charge = battery.remaining_capacity * 0.01
        msg.capacity = battery.nominal_capacity * 0.01
        msg.design_capacity = battery.nominal_capacity * 0.01
        if hasattr(msg, 'temperature'):
            msg.temperature = max_finite(battery.temperatures)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        self.battery_pub.publish(msg)

    def status(self, name):
        """Return the robot's state as a DiagnosticStatus (level, summary, every value)."""
        rcs, motion, sensor, safe, battery, height = self.snapshot()
        age = self.age()
        connected = age <= self.state_timeout
        with self._lock:
            feedback = dict(self._feedback)
        status = DiagnosticStatus(name=name, hardware_id=(rcs.robot_name if rcs and rcs.robot_name
                                                          else 'x30'))
        values = [('connected', connected), ('age_sec', '{:.2f}'.format(min(age, 1e6)))]
        problems, warnings = [], []
        if not connected:
            problems.append('no reports from {} {}'.format(
                self.link.remote[0], 'for {:.1f} s'.format(age) if motion is not None
                else 'yet (network.toml on the motion host?)'))
        if motion is not None:
            values += [
                ('basic_state', motion.basic_state),
                ('basic_state_name', self.state_name(motion.basic_state)),
                ('gait_state', motion.gait_state),
                ('gait_name', p.name_of(p.GAIT_STATES, motion.gait_state)),
                ('position_control', p.name_of(p.POSITION_CONTROL_STATES,
                                               motion.pos_ctrl_state)),
                ('distance_cm', '{:.0f}'.format(motion.robot_distance)),
                ('motion_layout', motion.layout),
            ]
            if motion.basic_state == p.STATE_EMERGENCY:
                warnings.append('emergency stop or fall')
            if not p.known_gait(motion.gait_state):
                values.append(('gait_speeds', 'not in Appendix B: scaled like walk'))
        if rcs is not None:
            values += [
                ('control_mode', 'non_manual' if rcs.non_manual else 'manual'),
                ('emergency_source', p.name_of(p.EMERGENCY_SOURCES, rcs.emergency_source)),
                ('errors', ' '.join(rcs.errors) or '-'),
                ('robot_name', rcs.robot_name or '-'),
                ('mileage_cm', rcs.current_mileage),
                ('total_mileage_cm', rcs.total_mileage),
                ('run_time_sec', rcs.current_run_time),
                ('motion_time_sec', rcs.current_motion_time),
                ('rcs_layout', rcs.layout),
            ]
            serious = {'driver_failure', 'imu_timeout'}
            if serious.intersection(rcs.errors):
                problems.extend(sorted(serious.intersection(rcs.errors)))
            others = [e for e in rcs.errors if e not in serious]
            if others:
                warnings.extend(others)
        values.append(('body_height', p.name_of(p.BODY_HEIGHTS, height)))
        if battery is not None:
            level = battery_fraction(battery.battery_level)
            values += [('battery', '{:.2f}'.format(level)),
                       ('battery_voltage', '{:.2f}'.format(
                           battery_voltage(battery.voltage, self.battery_voltage_scale)))]
            if level < self.low_battery:
                warnings.append('low battery')
        if safe is not None:
            values += [
                ('motor_temperature_max', '{:.1f}'.format(max_finite(safe.motor_temperature))),
                ('driver_temperature_max', max(safe.driver_temperature)),
                ('cpu_temperature', '{:.1f}'.format(safe.cpu_temperature)),
                ('cpu_frequency_mhz', '{:.0f}'.format(safe.cpu_frequency)),
            ]
        if sensor is not None:
            values.append(('imu_rpy_deg', ' '.join('{:.1f}'.format(v) for v in sensor.rpy_deg)))
        if self.perception is not None:
            values += [(key, self.feedback_name(key, value)) for key, value in feedback.items()]
        if self._ignored:
            warnings.append('cmd_vel ignored: ' + self._ignored)
            values.append(('cmd_vel_ignored', self._ignored))
        with self._policy_lock:
            sent = self.policy.sent_command(time.monotonic())
        values += [('velocity_mode', self.velocity_mode),
                   ('cmd', ' '.join('{:.2f}'.format(v) for v in sent)),
                   ('command_prefix', '0x{:02X}'.format(self.prefix))]
        if problems:
            status.level = DiagnosticStatus.ERROR
        elif warnings:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        summary = []
        if motion is not None and connected:
            summary.append(self.state_name(motion.basic_state))
            summary.append(p.name_of(p.GAIT_STATES, motion.gait_state))
        if rcs is not None and connected:
            summary.append('non-manual' if rcs.non_manual else 'manual')
        status.message = ', '.join(problems + warnings + summary) or 'ok'
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values]
        return status

    def publish_state(self, _event=None):
        """Publish the readable robot state."""
        self.state_pub.publish(self.status('x30'))

    def publish_diagnostics(self, _event=None):
        """Publish the driver's health."""
        status = self.status('x30_driver')
        status.values += [KeyValue(key=k, value=str(v)) for k, v in (
            ('rx', self.link.rx_count), ('tx', self.link.tx_count),
            ('tx_errors', self.link.tx_errors), ('rx_errors', self.link.rx_errors),
            ('last_error', self.link.last_error or '-'),
            ('unknown_codes', ' '.join('0x{:X}'.format(c) for c in sorted(self._unknown_codes))
             or '-'),
        )]
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def shutdown(self):
        """Stop the timers, send zero axes while stepping and close the links."""
        for timer in self._timers:
            timer.shutdown()
        try:
            self.zero_velocity()
        finally:
            self.link.stop()
            if self.perception is not None:
                self.perception.stop()


def main():
    """Run the driver node."""
    rospy.init_node('x30_driver')
    X30Driver()
    rospy.spin()
