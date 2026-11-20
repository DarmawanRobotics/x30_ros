"""fleet_uplink_node: the robot side of the NETRA command center (robot API v1, substation)."""

# Telemetry, patrol, stop-point and alert events over one WebSocket; taught routes and substation
# inspections through an on-disk spool uploaded over HTTP; NETRA's commands become the stack's
# services. NETRA's inspection plan of a patrol's route becomes x30_inspection's (netra/), and
# the reports of its points go back as NETRA's inspections.

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import os
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
import yaml

from x30_fleet.core import substation
from x30_fleet.core.config import FleetConfig
from x30_fleet.core.patrol import Patrol
from x30_fleet.core.routes import safe_route_id, summarize_csv
from x30_fleet.core.spool import Spool
from x30_fleet.utils.connection import NetraLink
from x30_fleet.utils.telemetry import FleetTelemetry
from x30_msgs.msg import InspectionReport
from x30_msgs.srv import LoadPath, LoadPathRequest
from x30_utils.io import expand, valid_name, write_text_atomic
from x30_utils.ros import EVENTS, X30Node, run
from x30_utils.ros.services import ServiceClient

NAME = 'fleet_uplink_node'
# NETRA command -> std_srvs/Trigger service of the stack
TRIGGERS = {
    'stop_patrol': 'navigation/stop',
    'pause': 'navigation/pause',
    'resume': 'navigation/resume',
    'localize': 'localization/start',
    'estop': 'control/estop/engage',
    'estop_release': 'control/estop/release',
    'dock': 'docking/dock',
    'undock': 'docking/undock',
    'recorder_start': 'recorder/start',  # the X30's camera recorder
    'recorder_stop': 'recorder/stop',
}
CALL_SEC = {'start': 30.0, 'inspect': 300.0}  # navigation/start waits for a pose
NETRA_DIR = 'netra'  # <sites_dir>/<site>/netra/<route_id>.csv and inspection.yaml
PLAN_FILE = 'inspection.yaml'
NOW = 'netra_now'  # the point of inspect_point: where the robot stands
MOVING = ('JOINING', 'FOLLOWING', 'APPROACHING', 'DWELLING', 'BLOCKED', 'OFF_PATH')


class FleetUplinkNode(X30Node):
    """Connects the robot to NETRA: telemetry, events, routes, inspections, commands."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = cfg = self.declare_config(FleetConfig).value
        self.robot = self.declare_robot().value
        self.spool = Spool(cfg.spool_dir)
        self.patrol = Patrol()
        self.plan: dict = {'points': []}  # x30_inspection's plan of NETRA's patrol route
        token = os.environ.get(cfg.token_env, '') if cfg.token_env else ''  # never a parameter
        self.link = NetraLink(cfg.server_url, token, cfg.robot_id, cfg.verify_tls, self.spool,
                              self.log)  # fmt: skip
        self.io = ThreadPoolExecutor(max_workers=4, thread_name_prefix='netra command')
        self.estop_pub = self.create_publisher(String, 'control/estop_request', EVENTS)
        self.triggers = {name: ServiceClient(Trigger, s) for name, s in TRIGGERS.items()}
        self.load_client = ServiceClient(LoadPath, 'navigation/load_path')
        self.start_client = ServiceClient(Trigger, 'navigation/start')
        self.inspect_client = ServiceClient(Trigger, 'inspection/inspect')
        self.telemetry = FleetTelemetry(self, cfg)
        self.create_subscription(String, 'route/saved', self.on_route_saved, EVENTS)
        self.create_subscription(InspectionReport, 'inspection/report', self.on_report, EVENTS)
        self.every(1.0 / cfg.telemetry_hz, self.send_telemetry)
        self.every(0.1, self.drain_commands)
        self.every(cfg.clock_sync_period_sec, self.sync_clock)
        self.add_shutdown_hook(self.close)
        if cfg.server_url:
            if not token:
                self.log.warning('no NETRA token: NETRA refuses the robot', variable=cfg.token_env)
            self.link.start(cfg.upload_period_sec)
            self.log.info('command center', url=cfg.server_url, robot_id=cfg.robot_id)
        else:
            self.log.warning('server_url empty: spooling uploads, not connecting')

    def close(self):
        """Stop the link's threads and leave the running calls."""
        self.link.stop()
        self.io.shutdown(wait=False)

    # ---------------------------------------------------------------- state
    def send(self, message: dict, durable: bool = False) -> bool:
        """Send to NETRA (durable messages wait while offline)."""
        return self.link.send(message, durable)

    def pose(self):
        """Return {x, y, yaw} in the map frame, or None."""
        frames = self.robot.frames
        found = self.tf.lookup(frames.map, frames.base)
        if found is None:
            return None
        pose = found.pose2d
        return {'x': round(pose.x, 3), 'y': round(pose.y, 3), 'yaw': round(pose.yaw, 3)}

    def send_telemetry(self):
        """Send the snapshot (telemetry_hz)."""
        clock = self.link.clock.report(time.time(), 3.0 * self.config.clock_sync_period_sec)
        self.send({'type': 'telemetry', 'pose': self.pose(), 'patrol': self.patrol.current,
                   'clock': clock, **self.patrol.context(), **self.telemetry.state})  # fmt: skip

    def sync_clock(self):
        """Measure the offset to NETRA's clock again; warn when ours is off."""
        self.link.sync_clock()
        best = self.link.clock.best()
        if best is not None and abs(best[0]) * 1000.0 > self.config.clock_warn_ms:
            self.log.warning('clock differs from NETRA (chrony on the robot)',
                             offset_ms=round(best[0] * 1000.0), throttle=60.0)  # fmt: skip

    # ---------------------------------------------------------------- uploads
    def on_route_saved(self, msg: String):
        """Spool a freshly taught route for NETRA's route database."""
        try:
            with open(msg.data, 'rb') as f:
                data = f.read()
            recorded_at = os.path.getmtime(msg.data)
        except OSError as error:
            self.log.error('cannot read the taught route', path=msg.data, error=str(error))
            return
        meta = {
            'name': os.path.splitext(os.path.basename(msg.data))[0],
            'recorded_at': recorded_at,
            'source_path': msg.data,
            'site': self.telemetry.site,
            **summarize_csv(data.decode(errors='ignore')),
        }
        self.spool.put('route', meta, {'route.csv': data})

    def on_report(self, msg: InspectionReport):
        """Spool the inspections of NETRA's points in a report (report.json and its images)."""
        try:
            with open(os.path.join(msg.directory, 'report.json'), encoding='utf-8') as f:
                report = json.load(f)
        except (OSError, ValueError) as error:
            self.log.error('cannot read the report', folder=msg.directory, error=str(error))
            return
        current = self.patrol.current or {}
        pose = self.pose() or {}
        found = substation.uploads(report, msg.directory, self.config.robot_id,
                                   current.get('patrol_id'), pose.get('yaw', 0.0))  # fmt: skip
        best = self.link.clock.best()
        for meta, paths in found:
            if best is not None:
                meta['ts'] = round(meta['ts'] + best[0], 3)  # onto NETRA's clock
            files = {}
            for field, path in paths.items():
                try:
                    with open(path, 'rb') as f:
                        files[field] = f.read()
                except OSError:
                    continue
            self.spool.put('inspection', meta, files)
            self.log.info('inspection spooled', point_id=meta['point_id'], uid=meta['uid'])

    # ---------------------------------------------------------------- commands
    def ack(self, cid, ok: bool, msg: str):
        """Acknowledge a command (from any thread)."""
        self.send({'type': 'ack', 'id': cid, 'ok': ok, 'msg': msg}, durable=True)

    def drain_commands(self):
        """Hand NETRA's commands to the worker threads."""
        while not self.link.incoming.empty():
            self.run_command(self.link.incoming.get())

    def run_command(self, message: dict):
        """Dispatch one command."""
        cid, name = message.get('id'), message.get('name')
        args = message.get('args') if isinstance(message.get('args'), dict) else {}
        if name == 'start_patrol':
            self.start_patrol(cid, args)
        elif name == 'inspect_point':
            self.io.submit(self.inspect_point, cid, args.get('point'))
        elif name in TRIGGERS:
            if name == 'estop':  # latch first, without waiting for the service
                self.estop_pub.publish(String(data='command center'))
            self.io.submit(self.trigger, cid, self.triggers[name])
        else:
            self.ack(cid, False, f'unknown command {name}')

    def trigger(self, cid, client: ServiceClient, timeout: float = 10.0) -> bool:
        """Call a Trigger service (worker thread); ack a failure, return True on success."""
        response, error = client.call(TriggerRequest(), timeout)
        if response is None or not response.success:
            self.ack(cid, False, error or response.message)
            return False
        self.ack(cid, True, response.message)
        return True

    def site_dir(self) -> str | None:
        """Return NETRA's folder of the active site, None without one."""
        site = self.telemetry.site
        if not valid_name(site):
            return None
        return os.path.join(expand(self.config.sites_dir), site, NETRA_DIR)

    def start_patrol(self, cid, args: dict):
        """Check the route and the site, then fetch, plan, load and start (worker thread)."""
        route_id, folder = args.get('route_id'), self.site_dir()
        if not safe_route_id(route_id):
            self.ack(cid, False, f'invalid route_id {route_id!r}')
            return
        if folder is None:
            self.ack(cid, False, 'no active site: load one first (site/load)')
            return
        self.io.submit(self.run_patrol, cid, args, folder)

    def run_patrol(self, cid, args: dict, folder: str):
        """Download the route and its plan into the site, load the route, start navigating."""
        route_id = str(args['route_id'])
        try:
            text = self.link.download_route(route_id, os.path.join(folder, f'{route_id}.csv'))
        except Exception as error:
            self.ack(cid, False, f'route download failed: {error}')
            return
        try:
            plan = self.link.inspection_plan(route_id)
        except Exception as error:  # a NETRA without the plan: the route runs without its points
            self.log.warning('no inspection plan from NETRA', route_id=route_id, error=str(error))
            plan = None
        if not isinstance(plan, dict):
            plan = {'points': []}
        stops = substation.route_stops(text.decode(errors='ignore'))
        self.plan, skipped = substation.to_plan(plan, stops)
        for why in skipped:
            self.log.warning('NETRA inspection point left out', detail=why)
        self.write_plan(folder, self.plan)
        relative = os.path.join(self.telemetry.site, NETRA_DIR, f'{route_id}.csv')
        response, error = self.load_client.call(LoadPathRequest(path=relative), 10.0)
        if response is None or not response.success:
            self.ack(cid, False, error or response.message)
            return
        response, error = self.start_client.call(TriggerRequest(), CALL_SEC['start'])
        if response is None or not response.success:
            self.ack(cid, False, error or response.message)
            return
        for event in self.patrol.start(args.get('patrol_id'), args.get('route_id')):
            self.send(event, durable=True)
        points = sum(len(p['targets']) for p in self.plan['points'])
        self.ack(cid, True, f'patrol started, {points} inspection points')

    def inspect_point(self, cid, point):
        """Take one NETRA point where the robot stands, now (worker thread)."""
        folder, here = self.site_dir(), self.pose()
        if folder is None or here is None:
            self.ack(cid, False, 'no active site' if folder is None else 'robot pose unknown')
            return
        if self.telemetry.state['nav']['state'] in MOVING:
            self.ack(cid, False, 'patrolling: the point is taken at its stop')
            return
        if not isinstance(point, dict) or substation.number(point.get('id')) is None:
            self.ack(cid, False, 'args.point is not an inspection point')
            return
        now = {'name': NOW, 'x': here['x'], 'y': here['y'], 'targets': [substation.target(point)]}
        self.write_plan(folder, {'points': [*self.plan['points'], now]})
        try:
            response, error = self.inspect_client.call(TriggerRequest(), CALL_SEC['inspect'])
        finally:
            self.write_plan(folder, self.plan)
        if response is None:
            self.ack(cid, False, error)
            return
        self.ack(cid, bool(response.success), response.message)

    def write_plan(self, folder: str, plan: dict):
        """Write x30_inspection's NETRA plan of the active site."""
        os.makedirs(folder, exist_ok=True)
        text = yaml.safe_dump(plan, sort_keys=False, default_flow_style=None)
        write_text_atomic(os.path.join(folder, PLAN_FILE), text)


def main():
    """Run the NETRA uplink."""
    run(FleetUplinkNode, NAME)


if __name__ == '__main__':
    main()
