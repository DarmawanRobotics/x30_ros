"""The services behind the handheld's commands, called on worker threads, and the patrol speed."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time

import rospy
from std_srvs.srv import Trigger, TriggerRequest

from x30_bridge.core import commands
from x30_bridge.core.routes import route_path
from x30_msgs.srv import (
    LoadPath,
    LoadPathRequest,
    MarkStopPoint,
    MarkStopPointRequest,
    SetMode,
    SetModeRequest,
)
from x30_utils.ros.services import ServiceClient

WORKERS = 4  # commands answered at the same time (an inspection takes minutes)
SPEED_POLL_SEC = 3.0  # the patrol speed is read this often while a handheld is connected


class CommandClients:
    """command name -> (client, request factory(args)); a factory raises ValueError on junk."""

    def __init__(self, config, speed_range):
        self.config = config
        self.speed_range = speed_range  # () -> (low, high) m/s of the patrol speed
        self.pool = ThreadPoolExecutor(max_workers=WORKERS, thread_name_prefix='teleop command')
        self.table: dict = {}
        for name, service in commands.TRIGGERS.items():
            self.add_trigger(name, service)
        self.table['set_mode'] = (
            ServiceClient(SetMode, 'control/set_mode'),
            lambda a: SetModeRequest(mode=commands.mode_of(a), source=config.remote_source),
        )
        self.table['mark_stop'] = (
            ServiceClient(MarkStopPoint, 'route/mark_stop_point'),
            lambda a: MarkStopPointRequest(dwell_sec=commands.dwell_seconds(a.get('dwell', 10.0))),
        )
        self.table['load_route'] = (
            ServiceClient(LoadPath, 'navigation/load_path'),
            lambda a: LoadPathRequest(path=route_path(config.sites_dir, a.get('file', ''))),
        )
        self.table['nav_speed'] = (None, self.speed_value)
        self.robot_commands = [name for name in config.robot_commands if name]
        prefix = config.robot_command_prefix.rstrip('/') + '/'
        for name in self.robot_commands:
            self.add_trigger(name, prefix + name)

    def add_trigger(self, name: str, service: str):
        """Add a std_srvs/Trigger command."""
        self.table[name] = (ServiceClient(Trigger, service), lambda _a: TriggerRequest())

    def names(self) -> list[str]:
        """Every command the welcome lists."""
        return sorted([*self.table, *commands.LOCAL])

    def speed_value(self, args: dict) -> float:
        """Return the patrol speed of a nav_speed command (ValueError outside the range)."""
        low, high = self.speed_range()
        return commands.patrol_speed(args.get('speed'), low, high)

    def run(self, name: str, request, done):
        """Call the command's service on a worker thread; done(ok, message) with its answer."""
        client = self.table[name][0]

        def call():
            if client is None:  # nav_speed: the follower's live parameter
                rospy.set_param(self.cruise_param(), float(request))
                done(True, f'patrol speed {request:g} m/s')
                return
            response, error = client.call(request, commands.call_sec(name))
            if response is None:
                done(False, error)
            else:
                done(bool(response.success), str(response.message))

        self.pool.submit(call)

    def cruise_param(self) -> str:
        """Return the follower's patrol speed parameter."""
        return self.config.navigation_node.rstrip('/') + '/speed/cruise'

    def close(self):
        """Let the running calls end on their own (they are bounded by their timeouts)."""
        self.pool.shutdown(wait=False)


class PatrolSpeed:
    """The follower's patrol speed for the handheld, read one request at a time."""

    def __init__(self, clients: CommandClients, speed_range):
        self.clients = clients
        self.speed_range = speed_range
        self.speed = None  # {target, min, max}; None without the parameter
        self.asked = -SPEED_POLL_SEC
        self.busy = False

    def due(self, now: float) -> bool:
        """Return True when the speed should be read again."""
        return not self.busy and now - self.asked >= SPEED_POLL_SEC

    def ask(self):
        """Read speed/cruise on a worker thread (the master answers, not the follower)."""
        self.asked, self.busy = time.monotonic(), True
        self.clients.pool.submit(self.read)

    def read(self):
        """Keep the follower's speed with the allowed range."""
        try:
            target = rospy.get_param(self.clients.cruise_param(), None)
        except Exception:  # the master went away meanwhile: read again later
            target = None
        finally:
            self.busy = False
        value = commands.number(target)
        if value is None:
            self.speed = None
            return
        low, high = self.speed_range()
        self.speed = {'target': round(value, 2), 'min': low, 'max': round(high, 2)}
