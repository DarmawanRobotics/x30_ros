#!/usr/bin/env python3
"""
A simulated X30: motion host and perception host on UDP.

The motion host's state machine and reports, the perception host's elevation map feedback and
autonomous charging. For the tests, and to try the driver without a robot:

    rosrun x30_driver fake_robot.py
    roslaunch x30_driver driver.launch \
        overrides_file:=$(rospack find x30_driver)/config/fake_robot.yaml
"""

import argparse
import math
import os
import select
import socket
import sys
import threading
import time

try:
    from x30_driver import protocol as p
except ImportError:  # run from the source tree without a catkin workspace
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
    from x30_driver import protocol as p


class FakeX30:
    """State machine of the motion host: posture, stepping, modes, gaits, height, axes."""

    TRANSITION_SEC = 0.6
    AXIS_TIMEOUT_SEC = 1.0  # 1.2.3: an axis command is invalid after one second

    def __init__(self, basic_state=p.STATE_SITTING, gait_state=p.GAIT_STATE_WALK,
                 non_manual=False, require_non_manual=True, layout='natural'):
        """Start in a basic state; autonomous (0x31) axis commands need non-manual mode."""
        self.basic_state = basic_state
        self.gait_state = gait_state
        self.non_manual = non_manual
        self.require_non_manual = require_non_manual
        self.layout = layout
        self.height = 0
        self.emergency_source = 0
        self.axes = [0, 0, 0]
        self.axes_time = None
        self.pose = [0.0, 0.0, 0.0]
        self.velocity = (0.0, 0.0, 0.0)
        self.now = 0.0
        self.heartbeats = 0
        self.connected = False
        self.saved = False
        self.received = []
        self.battery_level = 87
        self._pending = None  # (time, basic_state)

    # -------------------------------------------------------------- commands

    def handle(self, data: bytes):
        """Take one datagram as the motion host would."""
        d = p.parse_datagram(data)
        if d is None or d.kind != p.TYPE_SIMPLE:
            return
        full, value = d.code, p.signed(d.value)
        self.received.append(full)
        if full in (p.SAVE_DATA, p.SAVE_DATA_OLD):
            self.saved = True
            return
        base, prefix = full & 0xFFFFFF, full >> 24
        state = self.basic_state
        if base == p.HEARTBEAT:
            self.heartbeats += 1
        elif base == p.CONNECT:
            self.connected = True
        elif base == p.SOFT_ESTOP:
            self._set(p.STATE_EMERGENCY)
            self.emergency_source = 5  # client
        elif base == p.STAND_SIT and state == p.STATE_SITTING:
            self._set(p.STATE_SIT_TO_STAND)
            self._pending = (self.now + self.TRANSITION_SEC, p.STATE_INITIAL_STANDING)
        elif base == p.STAND_SIT and state in (p.STATE_INITIAL_STANDING,
                                               p.STATE_TORQUE_STANDING):
            self._set(p.STATE_STAND_TO_SIT)
            self._pending = (self.now + self.TRANSITION_SEC, p.STATE_SITTING)
        elif base == p.TORQUE_CONTROL and state == p.STATE_INITIAL_STANDING:
            self._set(p.STATE_TORQUE_STANDING)
        elif base == p.STEPPING and state == p.STATE_TORQUE_STANDING:
            self._set(p.STATE_STEPPING)
        elif base == p.STEPPING and state == p.STATE_STEPPING:
            self._set(p.STATE_TORQUE_STANDING)
        elif base == p.NON_MANUAL_MODE:
            self.non_manual = True
        elif base == p.MANUAL_MODE:
            self.non_manual = False
        elif base == p.BODY_HEIGHT:
            self.height = -1 if value == p.HEIGHT_CRAWL else 0
        elif base in p.GAIT_COMMANDS and state == p.STATE_STEPPING:
            self.gait_state = p.GAIT_COMMANDS[base]
        elif base in (p.AXIS_FORWARD, p.AXIS_LATERAL, p.AXIS_TURN):
            if state != p.STATE_STEPPING:
                return
            if prefix == p.PREFIX_AUTONOMOUS and self.require_non_manual and not self.non_manual:
                return
            index = (p.AXIS_FORWARD, p.AXIS_LATERAL, p.AXIS_TURN).index(base)
            self.axes[index] = p.clamp_axis(value)
            self.axes_time = self.now

    def _set(self, basic_state):
        self.basic_state = basic_state
        self._pending = None
        if basic_state != p.STATE_STEPPING:
            self.axes = [0, 0, 0]

    def codes(self, base=None):
        """Return the received full codes, or those with this command base."""
        if base is None:
            return list(self.received)
        return [c for c in self.received if c & 0xFFFFFF == base]

    # -------------------------------------------------------------- time

    def advance(self, now: float):
        """Finish transitions, run the axes as velocities and integrate the odometry."""
        dt = max(0.0, now - self.now)
        self.now = now
        if self._pending is not None and now >= self._pending[0]:
            target = self._pending[1]
            self._pending = None
            self.basic_state = target
        fresh = self.axes_time is not None and now - self.axes_time <= self.AXIS_TIMEOUT_SEC
        if self.basic_state == p.STATE_STEPPING and fresh:
            limits = p.gait_limits(self.gait_state, self.height == -1)
            self.velocity = p.axes_to_velocity(*self.axes, limits)
        else:
            self.velocity = (0.0, 0.0, 0.0)
        vx, vy, wz = self.velocity
        yaw = self.pose[2]
        self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.pose[2] = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))

    # -------------------------------------------------------------- reports

    def motion_report(self) -> bytes:
        """Return a MotionStateData datagram."""
        return p.encode_motion_state(p.MotionState(
            basic_state=self.basic_state, gait_state=self.gait_state,
            leg_odom_pos=tuple(self.pose), leg_odom_vel=self.velocity, robot_distance=1234.0,
            pos_ctrl_state=0,
        ), 'natural' if self.layout == 'natural' else 'pack1')

    def rcs_report(self) -> bytes:
        """Return an RcsData datagram."""
        return p.encode_rcs(p.RcsData(
            robot_name='X30-SIM', current_mileage=1234, total_mileage=567890,
            current_run_time=60, total_run_time=36000, current_motion_time=30,
            total_motion_time=18000, joystick=(0.0, 0.0, 0.0, 0.0),
            is_nav_mode=int(self.non_manual), emergency_source=self.emergency_source,
            error_state=0,
        ), 'natural' if self.layout == 'natural' else 'pack1')

    def sensor_report(self) -> bytes:
        """Return a ControllerSensorData datagram."""
        standing = self.basic_state in (p.STATE_INITIAL_STANDING, p.STATE_TORQUE_STANDING,
                                        p.STATE_STEPPING)
        pos = [0.0, -0.7, 1.4] * 4 if standing else [0.0, -1.2, 2.4] * 4
        return p.encode_sensor(p.SensorData(
            timestamp=int(self.now * 1000), rpy_deg=(0.5, -1.0, math.degrees(self.pose[2])),
            omega=(0.0, 0.0, self.velocity[2]), acc=(0.0, 0.0, 9.81),
            joint_pos=tuple(pos), joint_vel=(0.0,) * 12, joint_tau=(1.5,) * 12,
        ))

    def safe_report(self) -> bytes:
        """Return a ControllerSafeData datagram."""
        return p.encode_safe(p.SafeData(motor_temperature=(35.0,) * 12,
                                        driver_temperature=(40,) * 12,
                                        cpu_temperature=48.5, cpu_frequency=1800.0))

    def battery_report(self) -> bytes:
        """Return a BatterySensorData datagram."""
        return p.encode_battery(p.BatteryData(
            voltage=54, current=-350, remaining_capacity=1200, nominal_capacity=1500, cycles=12,
            production_date=0, balanced_low=0, balanced_high=0, protected_state=0,
            software_version=1, battery_level=self.battery_level, mos_state=0,
            battery_quantity=1, battery_ntc=4, temperatures=(28.0, 29.0, 30.5, 29.5),
        ), 'natural' if self.layout == 'natural' else 'pack1')

    def height_report(self) -> bytes:
        """Return the body height status."""
        return p.simple(p.BODY_HEIGHT_STATUS, self.height)


class FakePerception:
    """The perception host: elevation map modes with 10 Hz feedback, autonomous charging."""

    TRANSITION_SEC = 0.5

    def __init__(self):
        """Solid ground, decelerate, navigation source, idle charging."""
        self.elevation_mode = p.ELEVATION_SOLID_GROUND
        self.brake_mode = 1
        self.velocity_source = 2
        self.obstacle_height = 1
        self.charge_state = 0
        self.feedback_on = False
        self.now = 0.0
        self._pending = None  # (time, charge_state)

    def handle(self, data: bytes, elevation: bool):
        """Take one datagram; return the replies for the sender."""
        d = p.parse_datagram(data)
        if d is None or d.kind != p.TYPE_SIMPLE:
            return []
        value = p.signed(d.value)
        if elevation:
            self.feedback_on = True  # Appendix C: any message starts the feedback
            if d.code == p.ELEVATION_MODE and value in p.ELEVATION_MODES:
                self.elevation_mode = value
            elif d.code == p.BRAKE_MODE and value in p.BRAKE_MODES:
                self.brake_mode = value
            elif d.code == p.VELOCITY_SOURCE and value in p.VELOCITY_SOURCES:
                self.velocity_source = value
            elif d.code == p.OBSTACLE_HEIGHT and value in p.OBSTACLE_HEIGHTS:
                self.obstacle_height = value
            return []
        if d.code == p.CHARGE:
            if value == p.CHARGE_START:
                self.charge_state = 1
                self._pending = (self.now + self.TRANSITION_SEC, 2)
            elif value == p.CHARGE_STOP:
                self.charge_state = 3
                self._pending = (self.now + self.TRANSITION_SEC, 0)
            elif value == p.CHARGE_CLEAR:
                self.charge_state = 0
                self._pending = None
            return [p.simple(p.CHARGE, self.charge_state)]
        if d.code == p.CHARGE_QUERY:
            return [p.simple(p.CHARGE_QUERY, self.charge_state)]
        return []

    def advance(self, now: float):
        """Finish a charging transition."""
        self.now = now
        if self._pending is not None and now >= self._pending[0]:
            self.charge_state = self._pending[1]
            self._pending = None

    def feedback(self):
        """Return the 10 Hz elevation map feedback datagrams."""
        return [p.simple(p.ELEVATION_MODE_STATE, self.elevation_mode),
                p.simple(p.BRAKE_MODE_STATE, self.brake_mode),
                p.simple(p.VELOCITY_SOURCE_STATE, self.velocity_source)]


def udp_socket(host, port):
    """Return a non-blocking UDP socket bound to host:port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.setblocking(False)
    return sock


class FakeRobotServer:
    """Runs a FakeX30 and a FakePerception on UDP sockets."""

    def __init__(self, robot, perception, bind='127.0.0.1', motion_port=p.MOTION_HOST_PORT,
                 report_to=('127.0.0.1', 43897), vmap_port=p.ELEVATION_MAP_PORT,
                 charge_port=p.CHARGE_PORT, rate_hz=50.0, battery_hz=1.0):
        """Open the sockets; start() runs the loop on a thread."""
        self.robot = robot
        self.perception = perception
        self.report_to = report_to
        self.rate_hz = rate_hz
        self.battery_hz = battery_hz
        self.motion_sock = udp_socket(bind, motion_port)
        # port 0: any free port; None: no perception host
        self.vmap_sock = udp_socket(bind, vmap_port) if vmap_port is not None else None
        self.charge_sock = udp_socket(bind, charge_port) if charge_port is not None else None
        self.vmap_client = None
        self._running = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    @property
    def motion_port(self):
        """Return the bound motion host port."""
        return self.motion_sock.getsockname()[1]

    @property
    def vmap_port(self):
        """Return the bound elevation map port."""
        return self.vmap_sock.getsockname()[1] if self.vmap_sock else None

    @property
    def charge_port(self):
        """Return the bound charging port."""
        return self.charge_sock.getsockname()[1] if self.charge_sock else None

    def start(self):
        """Run the loop on a thread."""
        self._running.set()
        self._thread = threading.Thread(target=self.run, name='fake_x30', daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stop the loop and close the sockets."""
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for sock in (self.motion_sock, self.vmap_sock, self.charge_sock):
            if sock is not None:
                sock.close()

    def run(self):
        """Receive commands, advance the simulation and send reports at their rates."""
        start = time.monotonic()
        due = {'fast': 0.0, 'slow': 0.0, 'battery': 0.0, 'height': 0.0, 'feedback': 0.0}
        sockets = [s for s in (self.motion_sock, self.vmap_sock, self.charge_sock) if s]
        while self._running.is_set():
            readable, _, _ = select.select(sockets, [], [], 0.005)
            now = time.monotonic() - start
            with self._lock:
                for sock in readable:
                    try:
                        data, address = sock.recvfrom(65535)
                    except OSError:
                        continue
                    if sock is self.motion_sock:
                        self.robot.handle(data)
                    else:
                        elevation = sock is self.vmap_sock
                        if elevation:
                            self.vmap_client = address
                        for reply in self.perception.handle(data, elevation):
                            sock.sendto(reply, address)
                self.robot.advance(now)
                self.perception.advance(now)
                out = []
                if now >= due['fast']:
                    due['fast'] = now + 1.0 / self.rate_hz
                    out += [self.robot.motion_report(), self.robot.sensor_report(),
                            self.robot.rcs_report()]
                if now >= due['slow']:
                    due['slow'] = now + 1.0
                    out.append(self.robot.safe_report())
                if now >= due['battery']:
                    due['battery'] = now + 1.0 / self.battery_hz
                    out.append(self.robot.battery_report())
                if now >= due['height']:
                    due['height'] = now + 0.5
                    out.append(self.robot.height_report())
                feedback = []
                if self.perception.feedback_on and self.vmap_client and now >= due['feedback']:
                    due['feedback'] = now + 0.1
                    feedback = self.perception.feedback()
            for data in out:
                try:
                    self.motion_sock.sendto(data, self.report_to)
                except OSError:
                    pass
            for data in feedback:
                try:
                    self.vmap_sock.sendto(data, self.vmap_client)
                except OSError:
                    pass


def address(text):
    """Parse host:port."""
    host, _, port = text.rpartition(':')
    return host or '127.0.0.1', int(port)


def main(argv=None):
    """Run the simulated robot until interrupted."""
    parser = argparse.ArgumentParser(description='Simulated Jueying X30 (motion and perception '
                                                 'hosts) on UDP.')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--motion-port', type=int, default=p.MOTION_HOST_PORT)
    parser.add_argument('--report-to', type=address, default=('127.0.0.1', 43897),
                        help='where the reports go (the driver\'s local_ip:local_port)')
    parser.add_argument('--vmap-port', type=int, default=p.ELEVATION_MAP_PORT)
    parser.add_argument('--charge-port', type=int, default=p.CHARGE_PORT)
    parser.add_argument('--rate', type=float, default=50.0, help='Hz of the 200 Hz reports')
    parser.add_argument('--start', choices=('sitting', 'standing', 'stepping'),
                        default='sitting')
    parser.add_argument('--layout', choices=('natural', 'pack1'), default='natural')
    args, _ = parser.parse_known_args(argv)  # roslaunch adds __name:= and __log:=
    state = {'sitting': p.STATE_SITTING, 'standing': p.STATE_TORQUE_STANDING,
             'stepping': p.STATE_STEPPING}[args.start]
    server = FakeRobotServer(FakeX30(state, layout=args.layout), FakePerception(),
                             bind=args.bind, motion_port=args.motion_port,
                             report_to=args.report_to, vmap_port=args.vmap_port,
                             charge_port=args.charge_port, rate_hz=args.rate).start()
    print('fake X30: motion host {}:{}, reports to {}:{}, elevation map {}, charging {}'.format(
        args.bind, server.motion_port, args.report_to[0], args.report_to[1], server.vmap_port,
        server.charge_port))
    sys.stdout.flush()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
