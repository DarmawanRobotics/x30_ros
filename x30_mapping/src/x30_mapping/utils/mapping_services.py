"""mapping/start, mapping/stop and mapping/save of site_manager_node."""

from __future__ import annotations

from std_srvs.srv import TriggerRequest

from x30_mapping.core.session import StartPlan, plan_start
from x30_msgs.srv import LoadSiteRequest, StartMappingRequest
from x30_utils.io import PathError


def _needed(plan: StartPlan) -> list[str]:
    """Return the clients a plan calls."""
    names = ['load_tags', 'mapper_start']
    for flag, name in (
        (plan.reset_localization, 'reset'),
        (plan.localize, 'localize'),
        (plan.assume_origin, 'assume_origin'),
        (plan.resume, 'mapper_load'),
    ):
        if flag:
            names.append(name)
    return names


class MappingServices:
    """Mapping session services; mixed into SiteManagerNode (uses its state and clients)."""

    def _fail(self, response, site: str, message: str):
        response.success, response.message = False, message
        self.log.error('mapping/start refused', site=site, reason=message)
        return response

    def _on_mapping_start(self, request, response):
        """Service: map a site, new or existing, in its own frame."""
        site = request.site.strip()
        try:
            self.store.layout(site)
        except PathError as error:
            return self._fail(response, site, str(error))
        if self.mapping:
            return self._fail(response, site, f'already mapping {self.site}: mapping/stop first')
        has_content = self.store.has_content(site)
        plan = plan_start(
            site,
            request.reset,
            self.site,
            has_content,
            self.store.has_map(site),
            self.loc,
            self.frame_site,
        )
        missing = [n for n in _needed(plan) if not self.clients_[n].wait(1.0)]
        if missing:
            names = ', '.join(self.clients_[n].name for n in missing)
            return self._fail(response, site, f'not available: {names} (nothing changed)')
        notes, error = self._prepare(site, plan, has_content)
        error = error or self._start_mapper(site, plan.resume)
        if error:
            return self._fail(response, site, error)
        self.mapping = True
        verb = 'continuing the map of' if plan.resume else 'mapping'
        response.success = True
        response.message = self.site_message(verb, notes)
        return response

    def _start_mapper(self, site: str, resume: bool) -> str:
        """Load the saved map when resuming, start the grid mapper; return the error."""
        if resume:
            result = self.caller.call(self.clients_['mapper_load'], LoadSiteRequest(site=site))
            if not result.ok:
                return f'cannot continue the map: {result.message}'
        start = StartMappingRequest(site=site, reset=not resume)
        result = self.caller.call(self.clients_['mapper_start'], start)
        return '' if result.ok else f'grid mapper did not start: {result.message}'

    def _prepare(self, site: str, plan: StartPlan, has_content: bool) -> tuple[list, str]:
        """Archive, create, activate and localize as planned; return (notes, error)."""
        notes = []
        try:
            if plan.archive:
                notes.append(f'previous {site} moved to {self.store.archive(site)}')
            self.store.create(site)
        except OSError as error:
            return notes, f'cannot prepare {site}: {error}'
        if plan.activate:
            notes += self.activate(site, plan.reset_localization)
        if plan.localize:
            result = self.caller.call(self.clients_['localize'], TriggerRequest())
            if not result.ok:
                return notes, (
                    f'{site} has a map or tags already: let the robot see one of its tags, or '
                    f'give a pose estimate on its map, and retry; or start over with reset '
                    f'({result.message})'
                )
            self.frame_site = site
        ok, message = self.hand_tags(create=True)
        if not ok:
            return notes, f'localization cannot take the tags file: {message}'
        if plan.assume_origin:
            result = self.caller.call(self.clients_['assume_origin'], TriggerRequest())
            if not result.ok:
                return notes, f'cannot set the map origin: {result.message}'
            notes.append('map origin at the current pose')
        if plan.archive or not has_content:
            self.frame_site = site  # a new map: the current frame becomes the site's
        return notes, ''

    def _on_mapping_stop(self, request, response):
        """Service: stop integrating scans; /map shows the saved map again."""
        result = self.caller.call(self.clients_['mapper_stop'], TriggerRequest())
        if not result.ok and not result.unavailable:
            response.success = False
            response.message = f'grid mapper did not confirm the stop: {result.message}'
            self.log.error(response.message)
            return response
        was, self.mapping = self.mapping, False
        self.message = 'mapping stopped' if was else 'not mapping'
        if result.unavailable:
            self.message += ' (grid mapper not running)'
        if self.site:
            self.summary = self.store.summary(self.site)
        self.publish_site()
        response.success, response.message = True, self.message
        return response

    def _on_mapping_save(self, request, response):
        """Service: write the grid into the active site and publish it as the site map."""
        if not self.site:
            response.success, response.message = False, 'no site: call mapping/start first'
            return response
        result = self.caller.call(self.clients_['mapper_save'], LoadSiteRequest(site=self.site))
        if not result.ok:
            response.success, response.message = False, result.message
            self.log.error('mapping/save failed', site=self.site, reason=result.message)
            return response
        self.summary = self.store.summary(self.site)
        self.message = f'map of {self.site} saved'
        self.publish_site()
        response.success, response.message = True, result.message
        return response
