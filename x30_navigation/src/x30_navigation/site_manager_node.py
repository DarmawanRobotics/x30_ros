"""site_manager_node: the active site (its tags and routes), loaded at start or on request."""

from __future__ import annotations

from std_srvs.srv import Trigger, TriggerRequest

from x30_msgs.msg import SiteStatus
from x30_msgs.srv import ListSites, LoadFile, LoadFileRequest, LoadSite
from x30_navigation.core.config.sites import SiteConfig
from x30_navigation.core.sites.store import SiteStore, SiteSummary, valid_name
from x30_navigation.utils.calls import Caller
from x30_utils.io import expand, write_text_atomic
from x30_utils.ros import X30Node, run

NAME = 'site_manager_node'


class SiteManagerNode(X30Node):
    """Own the active site and every file of it: its tags go to localization."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(SiteConfig).value
        self.store = SiteStore(self.config.sites_dir)
        self.state_file = expand(self.config.state_file)
        self.site = ''
        self.summary = SiteSummary('')
        self.tags_pending = False
        self.message = 'no site loaded'
        self._last_status = None
        self.caller = Caller(self, self.config.service_timeout_sec)
        self._status_pub = self.latched(SiteStatus, 'site/status')
        self.clients_ = {
            'load_tags': self.caller.client(LoadFile, 'localization/load_tags'),
            'reset': self.caller.client(Trigger, 'localization/reset'),
        }
        # one site operation at a time
        self.create_service(ListSites, 'site/list', self._on_list)
        self.create_service(LoadSite, 'site/load', self._on_load)
        self.after(0.1, self._on_startup)
        self.every(self.config.refresh_sec, self._refresh)
        self.publish_status(force=True)
        self.log.info('site manager ready', sites_dir=self.store.root)

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

    def hand_tags(self) -> tuple[bool, str]:
        """Give localization the active site's tags file; retried later when it is absent."""
        path = self.store.layout(self.site).tags
        result = self.caller.call(
            self.clients_['load_tags'], LoadFileRequest(path=path, create=False)
        )
        self.tags_pending = result.unavailable or not result.ok
        return result.ok, result.message

    def activate(self, site: str, reset_localization: bool) -> list[str]:
        """Make site active (its tags handed to localization, its routes); return notes."""
        self.summary = self.store.summary(site)
        notes = list(self.summary.problems)
        for problem in notes:
            self.log.warning('site file problem', site=site, problem=problem)
        changed = site != self.site
        self.site = site
        if changed:
            self.remember(site)
        if reset_localization and changed:
            result = self.caller.call(self.clients_['reset'], TriggerRequest())
            if not result.ok:
                notes.append(f'localization not reset: {result.message}')
        self.message = f'loading {site}'
        ok, message = self.hand_tags()
        if not ok:
            notes.append(f'tags not loaded yet: {message}')
        self.publish_status(force=True)
        return notes

    def site_message(self, verb: str, notes: list[str]) -> str:
        """Log, store and return the outcome of a site change."""
        s = self.summary
        message = f'{verb} {self.site}: {s.tags} tags, {len(s.routes)} routes'
        if notes:
            message += ' (' + '; '.join(notes) + ')'
        self.message = message
        self.log.info(message)
        self.publish_status(force=True)
        return message

    # ------------------------------------------------------------------ publishing

    def publish_status(self, force: bool = False):
        """Publish site/status when it changed (or always with force)."""
        values = (
            self.site,
            tuple(self.store.list_sites()),
            self.summary.tags,
            len(self.summary.routes),
            self.message,
        )
        if values == self._last_status and not force:
            return
        self._last_status = values
        msg = SiteStatus(site=values[0], sites=list(values[1]))
        msg.tags, msg.routes, msg.message = values[2:]
        msg.header.stamp = self.stamp()
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------ timers

    def _on_startup(self):
        """Load the site of the parameter or of the last run, once the node is up."""
        site = self.config.site.strip() or self.remembered()
        if not site:
            self.log.warning('no site yet: give one (site:=), routes are refused')
            return
        if not self.store.exists(site) and not self._create_at_start(site):
            self.message = f'site {site!r} not found in {self.store.root}'
            self.log.warning(self.message)
            self.publish_status(force=True)
            return
        self.site_message('loaded', self.activate(site, reset_localization=False))

    def _create_at_start(self, site: str) -> bool:
        """Create the launch site, an empty folder for its routes; False: not asked or invalid."""
        if not self.config.create_site or site != self.config.site.strip() or not valid_name(site):
            return False
        try:
            self.store.create(site)
        except OSError as error:
            self.log.error('site not created', site=site, error=str(error))
            return False
        self.log.info('site created: its routes and tags go there', site=site)
        return True

    def _refresh(self):
        """Retry the tags handover; republish the status when files changed."""
        if self.tags_pending and self.site and self.clients_['load_tags'].ready():
            ok, message = self.hand_tags()
            if ok:
                self.log.info('tags handed to localization', site=self.site, result=message)
        if self.site:
            self.summary.tags, _ = self.store.tags(self.site)
            self.summary.routes = self.store.routes(self.site)
        self.publish_status()

    # ------------------------------------------------------------------ site services

    def _on_list(self, request, response):
        response.sites = self.store.list_sites()
        response.active = self.site
        return response

    def _on_load(self, request, response):
        """Service: make a site active (another site resets localization)."""
        site = request.site.strip()
        if not self.store.exists(site):
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
