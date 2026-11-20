"""site_manager_node: the active site (map, fiducials, routes), /map and mapping sessions."""

from __future__ import annotations

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, TriggerRequest

from x30_mapping.core.config import SiteConfig
from x30_mapping.core.session import Localization
from x30_mapping.core.site_store import SiteStore, SiteSummary, valid_name
from x30_mapping.utils.calls import Caller
from x30_mapping.utils.conversions import grid_msg, localization
from x30_mapping.utils.mapping_services import MappingServices
from x30_msgs.msg import LocalizationStatus, SiteStatus
from x30_msgs.srv import (
    ListSites,
    LoadFile,
    LoadFileRequest,
    LoadSite,
    StartMapping,
    StartMappingRequest,
    StartMappingResponse,
)
from x30_utils.io import expand, write_text_atomic
from x30_utils.ros import LATCHED, X30Node, run

NAME = 'site_manager_node'


class SiteManagerNode(MappingServices, X30Node):
    """Own the active site and every file of it; orchestrate mapping sessions."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(SiteConfig).value
        self.frames = self.declare_robot().value.frames
        self.store = SiteStore(self.config.sites_dir)
        self.state_file = expand(self.config.state_file)
        self.site = ''
        self.summary = SiteSummary('')
        self.mapping = False
        self.loc = Localization()
        self.frame_site = ''  # site whose frame the current localization was adopted for
        self.tags_pending = False
        self.message = 'no site loaded'
        self._last_status = None
        self.caller = Caller(self, self.config.service_timeout_sec)
        self._map_pub = self.latched(OccupancyGrid, '/map')
        self._status_pub = self.latched(SiteStatus, 'site/status')
        self.create_subscription(
            LocalizationStatus, 'localization/status', self._on_localization, LATCHED
        )
        self.clients_ = self._make_clients()
        services = [
            (ListSites, 'site/list', self._on_list),
            (LoadSite, 'site/load', self._on_load),
        ]
        if self.config.mapping:  # a robot without a map offers no mapping session
            self.create_subscription(OccupancyGrid, 'mapping/grid', self._on_grid, LATCHED)
            services += [
                (StartMapping, 'mapping/start', self._on_mapping_start),
                (Trigger, 'mapping/stop', self._on_mapping_stop),
                (Trigger, 'mapping/save', self._on_mapping_save),
            ]
        for srv_type, name, callback in services:
            self.create_service(srv_type, name, callback)  # one site operation at a time
        self.after(0.1, self._on_startup)
        self._auto_mapping = None
        self.every(self.config.refresh_sec, self._refresh)
        self.publish_status(force=True)
        self.log.info('site manager ready', sites_dir=self.store.root)

    def _make_clients(self) -> dict:
        client = self.caller.client
        return {
            'load_tags': client(LoadFile, 'localization/load_tags'),
            'assume_origin': client(Trigger, 'localization/assume_origin'),
            'reset': client(Trigger, 'localization/reset'),
            'localize': client(Trigger, 'localization/start'),
            'mapper_start': client(StartMapping, 'grid_mapper/start'),
            'mapper_stop': client(Trigger, 'grid_mapper/stop'),
            'mapper_save': client(LoadSite, 'grid_mapper/save'),
            'mapper_load': client(LoadSite, 'grid_mapper/load'),
        }

    # ------------------------------------------------------------------ site state

    def remember(self, site: str):
        """Store the active site for the next start."""
        try:
            write_text_atomic(self.state_file, site + '\n')
        except OSError as error:
            self.log.warning('cannot remember the site', path=self.state_file, error=str(error))

    def remembered(self) -> str:
        """Return the site of the last run, or ''."""
        try:
            with open(self.state_file, encoding='utf-8') as f:
                site = f.read().strip()
        except OSError:
            return ''
        return site if valid_name(site) else ''

    def hand_tags(self, create: bool) -> tuple[bool, str]:
        """Give localization the active site's tags file; retried later when it is absent."""
        path = self.store.layout(self.site).tags
        result = self.caller.call(
            self.clients_['load_tags'], LoadFileRequest(path=path, create=create)
        )
        self.tags_pending = result.unavailable or (not result.ok and not create)
        return result.ok, result.message

    def activate(self, site: str, reset_localization: bool) -> list[str]:
        """Make site active (map, fiducials, routes, tags handed over); return notes."""
        self.summary = self.store.summary(site)
        notes = list(self.summary.problems)
        for problem in notes:
            self.log.warning('site file problem', site=site, problem=problem)
        changed = site != self.site
        self.site = site
        if changed:
            self.remember(site)
        if site != self.frame_site:
            self.frame_site = ''
        if reset_localization and changed:
            result = self.caller.call(self.clients_['reset'], TriggerRequest())
            if result.ok:
                self.loc = Localization()  # the latched status follows a moment later
            else:
                notes.append(f'localization not reset: {result.message}')
        self.message = f'loading {site}'
        ok, message = self.hand_tags(create=False)
        if not ok:
            notes.append(f'tags not loaded yet: {message}')
        self.publish_site()
        return notes

    def site_message(self, verb: str, notes: list[str]) -> str:
        """Log, store and return the outcome of a site change."""
        s = self.summary
        message = f'{verb} {self.site}: {"map" if s.map is not None else "no map yet"}'
        message += f', {s.tags} tags, {len(s.routes)} routes'
        if notes:
            message += ' (' + '; '.join(notes) + ')'
        self.message = message
        self.log.info(message)
        self.publish_status(force=True)
        return message

    # ------------------------------------------------------------------ publishing

    def publish_site(self):
        """Publish the site map (unless mapping) and the status."""
        if not self.mapping:
            stamp = self.stamp()
            self._map_pub.publish(grid_msg(self.summary.map, self.frames.map, stamp))
        self.publish_status(force=True)

    def publish_status(self, force: bool = False):
        """Publish site/status when it changed (or always with force)."""
        values = (
            self.site,
            tuple(self.store.list_sites()),
            self.summary.map is not None,
            self.mapping,
            self.summary.tags,
            len(self.summary.routes),
            self.message,
        )
        if values == self._last_status and not force:
            return
        self._last_status = values
        msg = SiteStatus(site=values[0], sites=list(values[1]), has_map=values[2])
        msg.mapping, msg.tags, msg.routes, msg.message = values[3:]
        msg.header.stamp = self.stamp()
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------ timers, inputs

    def _on_startup(self):
        """Load the site of the parameter or of the last run, once the node is up."""
        if self.config.start_mapping and self._start_mapping_soon():
            return
        site = self.config.site.strip() or self.remembered()
        if not site:
            if self.config.create_site:
                self.log.warning('create_site: no site given (site:=), routes are refused')
            elif self.config.mapping:
                self.log.info('no site yet: mapping/start creates one')
            else:
                self.log.warning('no site yet: give one (site:=), routes are refused')
            return
        if not self.store.exists(site) and not self._create_at_start(site):
            self.message = f'site {site!r} not found in {self.store.root}'
            self.log.warning(self.message)
            self.publish_status(force=True)
            return
        self.site_message('loaded', self.activate(site, reset_localization=False))

    def _create_at_start(self, site: str) -> bool:
        """Create the launch site without a map (odometry only); False: not asked or invalid."""
        if not self.config.create_site or site != self.config.site.strip() or not valid_name(site):
            return False
        try:
            self.store.create(site)
        except OSError as error:
            self.log.error('site not created', site=site, error=str(error))
            return False
        self.log.info('site created: no map, its routes and tags', site=site)
        return True

    def _start_mapping_soon(self) -> bool:
        """Map the launch site at start (mapping mode); False when it is not a new site."""
        site = self.config.site.strip()
        if not valid_name(site) or self.store.has_content(site):
            self.log.warning(
                'start_mapping: not a new site, loading it instead '
                '(mapping/start continues or resets an existing site)',
                site=site,
            )
            return False
        self._auto_mapping = self.every(1.0, self._auto_start)
        return True

    def _auto_start(self):
        needed = ('load_tags', 'assume_origin', 'mapper_start')
        if not all(self.clients_[n].ready() for n in needed):
            return
        self._auto_mapping.shutdown()
        request = StartMappingRequest(site=self.config.site.strip(), reset=True)
        result = self._on_mapping_start(request, StartMappingResponse())
        (self.log.info if result.success else self.log.error)(f'start_mapping: {result.message}')

    def _refresh(self):
        """Retry the tags handover; republish the status when files changed."""
        if self.tags_pending and self.site and self.clients_['load_tags'].ready():
            ok, message = self.hand_tags(create=False)
            if ok:
                self.log.info('tags handed to localization', site=self.site, result=message)
        if self.site:
            self.summary.tags, _ = self.store.tags(self.site)
            self.summary.routes = self.store.routes(self.site)
        self.publish_status()

    def _on_grid(self, msg: OccupancyGrid):
        """Relay the live grid to /map while mapping."""
        if self.mapping:
            self._map_pub.publish(msg)

    def _on_localization(self, msg: LocalizationStatus):
        self.loc = localization(msg)
        if not self.loc.localized:
            self.frame_site = ''  # the frame adopted for a new site is gone

    # ------------------------------------------------------------------ site services

    def _on_list(self, request, response):
        response.sites = self.store.list_sites()
        response.active = self.site
        return response

    def _on_load(self, request, response):
        """Service: make a site active (another site resets localization)."""
        site = request.site.strip()
        if self.mapping:
            response.success, response.message = False, 'mapping: call mapping/stop first'
        elif not self.store.exists(site):
            response.success, response.message = False, f'no site {site!r}'
        else:
            notes = self.activate(site, reset_localization=True)
            response.success, response.message = True, self.site_message('loaded', notes)
        return response


def main():
    """Run site_manager_node."""
    run(SiteManagerNode, NAME)


if __name__ == '__main__':
    main()
