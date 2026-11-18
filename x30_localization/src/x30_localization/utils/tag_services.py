"""The tags file services load_tags and record_tags (record_tags waits beside the timers)."""

from __future__ import annotations

import os
import threading

from std_srvs.srv import Trigger

from x30_localization.core.tags import load_tags
from x30_msgs.srv import LoadFile
from x30_utils.io import PathError, resolve_inside, write_tags

RECORD_MARGIN_SEC = 3.0


class TagServices:
    """The active tags file of the node and the services around it."""

    def __init__(self, node):
        self.node = node
        self.path = ''
        self.done = threading.Event()
        self.result = (False, '')
        node.create_service(LoadFile, 'localization/load_tags', self.srv_load)
        # it waits for the timer that finishes the recording: beside it, not after it
        node.create_service(Trigger, 'localization/record_tags', self.srv_record, concurrent=True)

    def tick(self, now: float):
        """End a finished recording (called by the TF timer)."""
        node = self.node
        with node.lock:
            session = node.localizer.session
            if session is None or not session.done(now):
                return
            self.result = node.localizer.finish_record(self.path)
        node.publish_markers()
        self.done.set()

    def srv_load(self, request, response):
        """Switch the tags file (path inside sites_dir); create starts an empty one."""
        raw = request.path.strip()
        if not raw:
            response.success, response.message = False, 'empty path'
            return response
        try:
            path = resolve_inside(self.node.config.sites_dir, raw)
        except PathError as error:
            response.success, response.message = False, str(error)
            self.node.log.error('load_tags refused', reason=str(error))
            return response
        if request.create and not os.path.exists(path):
            try:
                write_tags(path, {})
            except (OSError, ValueError) as error:
                response.success, response.message = False, f'cannot create {path}: {error}'
                return response
        response.success, response.message = self.load(path)
        return response

    def load(self, path: str) -> tuple[bool, str]:
        """Use path as the tags file (missing: no tags yet; broken entries are reported)."""
        tags, problems = load_tags(path)
        for problem in problems:
            self.node.log.error('tags file problem', problem=problem)
        with self.node.lock:
            self.path = path
            self.node.localizer.tags = tags
        self.node.publish_markers()
        message = f'{len(tags)} tag(s) from {path or "(no file)"}'
        if problems:
            message += f'; {len(problems)} problem(s): ' + '; '.join(problems)
        self.node.log.info('tags file', path=path, tags=len(tags), problems=len(problems))
        return True, message

    def srv_record(self, _request, response):
        """Record the map pose of the visible tags over record_duration_sec."""
        node = self.node
        with node.lock:
            self.done.clear()
            reason = node.localizer.start_record(node.config.tag.frames, node.now())
        if reason:
            response.success, response.message = False, f'record_tags refused: {reason}'
            node.log.error(response.message)
            return response
        timeout = node.config.tag.record_duration_sec + RECORD_MARGIN_SEC
        if not self.done.wait(timeout):
            with node.lock:
                node.localizer.session = None
            response.success, response.message = False, 'recording did not finish (TF stalled?)'
            return response
        response.success, response.message = self.result
        (node.log.info if response.success else node.log.error)(response.message)
        return response
