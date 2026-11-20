"""In-process test of site_manager_node against fake localization and grid mapper services."""

from __future__ import annotations

import itertools
import os
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rospy
from std_srvs.srv import Trigger

from x30_mapping.core import map_io
from x30_mapping.site_manager_node import SiteManagerNode
from x30_msgs.msg import LocalizationStatus, SiteStatus
from x30_msgs.srv import ListSites, LoadFile, LoadSite, StartMapping
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, delete_params, wait_for

NAMES = itertools.count()


class FakeRobot:
    """Fake localization and grid mapper services that record their calls."""

    def __init__(self, node, sites_dir):
        self.node, self.sites_dir = node, sites_dir
        self.calls = []
        self.localize_ok = False
        self.stop_delay = 0.0
        self.status = node.latched(LocalizationStatus, 'localization/status')
        self.set_status(LocalizationStatus.NONE, '')
        self.services = {
            name: node.create_service(
                srv, name, handler, concurrent=True
            )  # a slow one blocks none
            for srv, name, handler in (
                (LoadFile, 'localization/load_tags', self.load_tags),
                (Trigger, 'localization/assume_origin', self.assume_origin),
                (Trigger, 'localization/reset', self.reset),
                (Trigger, 'localization/start', self.localize),
                (StartMapping, 'grid_mapper/start', self.mapper_start),
                (Trigger, 'grid_mapper/stop', self.mapper_stop),
                (LoadSite, 'grid_mapper/save', self.mapper_save),
                (LoadSite, 'grid_mapper/load', self.mapper_load),
            )
        }

    def set_status(self, state, source):
        self.status.publish(LocalizationStatus(state=state, source=source))

    def names(self):
        return [c[0] for c in self.calls]

    def load_tags(self, request, response):
        self.calls.append(('load_tags', request.path, request.create))
        if request.create and not os.path.exists(request.path):
            with open(request.path, 'w') as f:
                f.write('{}')
        response.success, response.message = True, 'ok'
        return response

    def assume_origin(self, _request, response):
        self.calls.append(('assume_origin',))
        self.set_status(LocalizationStatus.LOCALIZED, 'origin')
        response.success = True
        return response

    def reset(self, _request, response):
        self.calls.append(('reset',))
        self.set_status(LocalizationStatus.NONE, '')
        response.success = True
        return response

    def localize(self, _request, response):
        self.calls.append(('localize',))
        response.success = self.localize_ok
        response.message = 'tag base_map' if self.localize_ok else 'no fresh detection'
        if self.localize_ok:
            self.set_status(LocalizationStatus.LOCALIZED, 'tag:base_map')
        return response

    def mapper_start(self, request, response):
        self.calls.append(('mapper_start', request.site, request.reset))
        response.success = True
        return response

    def mapper_stop(self, _request, response):
        self.calls.append(('mapper_stop',))
        time.sleep(self.stop_delay)
        response.success = True
        return response

    def mapper_save(self, request, response):
        self.calls.append(('mapper_save', request.site))
        data = np.zeros((20, 30), dtype=np.int8)
        data[0, :] = 100
        map_io.save_map(
            os.path.join(self.sites_dir, request.site), map_io.GridMap(data, 0.1, -1, -1)
        )
        response.success, response.message = True, 'saved'
        return response

    def mapper_load(self, request, response):
        self.calls.append(('mapper_load', request.site))
        response.success = True
        return response


def write_site(
    sites_dir,
    site,
    tags='{"base_map": {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}}',
):
    root = os.path.join(sites_dir, site)
    os.makedirs(os.path.join(root, 'routes'), exist_ok=True)
    data = np.zeros((10, 10), np.int8)
    map_io.save_map(root, map_io.GridMap(data, 0.05, 0.0, 0.0))
    with open(os.path.join(root, 'tags.json'), 'w') as f:
        f.write(tags)
    return root


class Rig:
    """The site manager, the fake services and the collectors, in this process."""

    def __init__(self, tmp_path, **params):
        self.closed = False
        self.ns = f'/site_test_{next(NAMES)}'
        self.sites = str(tmp_path / 'sites')
        os.makedirs(self.sites, exist_ok=True)
        defaults = {
            'sites_dir': self.sites,
            'state_file': str(tmp_path / 'state' / 'site'),
            'service_timeout_sec': 1.0,
            'refresh_sec': 0.5,
        }
        self.helper = X30Node('helper', param_ns=f'{self.ns}/helper')
        self.robot = FakeRobot(self.helper, self.sites)
        self.node = SiteManagerNode(
            param_ns=f'{self.ns}/site_manager_node', overrides={**defaults, **params}
        )
        self.status = Collector(SiteStatus, 'site/status')
        self.maps = Collector(OccupancyGrid, '/map')

    def call(self, srv, name, **request):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, srv)(**request)

    def start(self, site, reset=False):
        return self.call(StartMapping, 'mapping/start', site=site, reset=reset)

    def trigger(self, name):
        return self.call(Trigger, name)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.status.close()
        self.maps.close()
        self.node.destroy_node()
        self.helper.destroy_node()
        delete_params(self.ns)


def says(rig: Rig, text: str):
    """Return a check that the last site/status message holds text."""
    return lambda: rig.status.last is not None and text in rig.status.last.message


@pytest.fixture
def make_rig(ros, tmp_path):
    rigs = []

    def make(**params):
        rigs.append(Rig(tmp_path, **params))
        return rigs[-1]

    yield make
    for r in rigs:
        r.close()


def test_new_site_is_mapped_saved_and_published(make_rig):
    rig = make_rig()
    started = rig.start('lobby', reset=True)
    assert started.success, started.message
    names = rig.robot.names()
    assert names.index('load_tags') < names.index('assume_origin') < names.index('mapper_start')
    assert ('mapper_start', 'lobby', True) in rig.robot.calls
    assert wait_for(lambda: rig.status.last.mapping and rig.status.last.site == 'lobby', 3.0)
    assert not rig.start('hall').success  # already mapping
    assert not rig.call(LoadSite, 'site/load', site='lobby').success
    saved = rig.trigger('mapping/save')
    assert saved.success, saved.message
    assert ('mapper_save', 'lobby') in rig.robot.calls
    assert rig.trigger('mapping/stop').success
    assert wait_for(lambda: rig.maps.last is not None and rig.maps.last.info.width == 30, 3.0)
    assert wait_for(lambda: not rig.status.last.mapping, 3.0)  # its own connection: may be later
    status = rig.status.last
    assert status.has_map and not status.mapping and status.sites == ['lobby']
    listed = rig.call(ListSites, 'site/list')
    assert listed.sites == ['lobby'] and listed.active == 'lobby'
    # the frame adopted for the new site counts while localization lasts
    resumed = rig.start('lobby')
    assert resumed.success, resumed.message
    assert ('mapper_load', 'lobby') in rig.robot.calls and 'localize' not in rig.robot.names()


def test_origin_at_boot_does_not_continue_an_existing_site(make_rig):
    """A remembered site with a map is not continued on an 'origin' localization."""
    rig = make_rig()
    write_site(rig.sites, 'lobby')
    rig.close()
    rig = make_rig(site='lobby')
    assert wait_for(lambda: rig.status.last and 'loaded lobby' in rig.status.last.message, 5.0)
    rig.robot.set_status(LocalizationStatus.LOCALIZED, 'origin')
    time.sleep(0.3)
    refused = rig.start('lobby')
    assert not refused.success and 'tags' in refused.message
    assert 'localize' in rig.robot.names() and 'mapper_start' not in rig.robot.names()
    assert os.path.exists(os.path.join(rig.sites, 'lobby', 'map.yaml'))  # nothing archived
    rig.robot.localize_ok = True
    continued = rig.start('lobby')
    assert continued.success, continued.message
    assert ('mapper_start', 'lobby', False) in rig.robot.calls


def test_reset_archives_only_when_it_can_start(make_rig):
    """Missing services refuse before archiving."""
    rig = make_rig()
    root = write_site(rig.sites, 'lobby')
    rig.robot.services['grid_mapper/start'].shutdown()
    refused = rig.start('lobby', reset=True)
    assert not refused.success and 'nothing changed' in refused.message
    assert os.path.exists(os.path.join(root, 'map.yaml'))
    assert not [n for n in os.listdir(root) if n.startswith('backup')]


def test_reset_archives_into_unique_directories(make_rig):
    rig = make_rig()
    root = write_site(rig.sites, 'lobby')
    for _ in range(2):
        assert rig.start('lobby', reset=True).success
        assert rig.trigger('mapping/save').success
        assert rig.trigger('mapping/stop').success
    backups = sorted(n for n in os.listdir(root) if n.startswith('backup'))
    assert len(backups) == 2
    assert os.path.exists(os.path.join(root, backups[0], 'map.yaml'))
    assert os.path.exists(os.path.join(root, backups[1], 'map.yaml'))


def test_timed_out_stop_is_a_failure(make_rig):
    """A mapper that does not confirm the stop keeps the session (site/load refused)."""
    rig = make_rig()
    assert rig.start('lobby', reset=True).success
    rig.robot.stop_delay = 2.0
    stopped = rig.trigger('mapping/stop')
    assert not stopped.success and 'timed out' in stopped.message
    assert rig.node.mapping
    rig.robot.stop_delay = 0.0
    assert rig.trigger('mapping/stop').success


def test_corrupt_site_files_do_not_crash_the_boot(make_rig):
    """Broken tags and map are notes in the status; the node keeps running."""
    rig = make_rig()
    root = write_site(rig.sites, 'lobby', tags='{broken')
    with open(os.path.join(root, 'map.pgm'), 'wb') as f:
        f.write(b'junk')
    rig.close()
    rig = make_rig(site='lobby')
    loaded = lambda: rig.status.last and 'loaded lobby' in rig.status.last.message  # noqa: E731
    assert wait_for(loaded, 5.0)
    status = rig.status.last
    assert not status.has_map and status.tags == 0
    assert 'map not loaded' in status.message and 'tags.json' in status.message
    assert rig.call(ListSites, 'site/list').active == 'lobby'


def test_odometry_only_creates_its_launch_site_without_a_map(make_rig):
    """create_site: the launch site as an empty folder for routes and tags; never a bad name."""
    rig = make_rig(site='gardu', create_site=True)
    assert wait_for(lambda: rig.status.last and rig.status.last.site == 'gardu', 5.0)
    assert os.path.isdir(os.path.join(rig.sites, 'gardu', 'routes'))
    assert not rig.status.last.has_map and 'loaded gardu' in rig.status.last.message
    assert rig.robot.names().count('reset') == 0  # the origin localization stays
    rig.close()
    for site, create in (('Bad Name', True), ('missing', False)):
        refused = make_rig(site=site, create_site=create)
        assert wait_for(says(refused, 'not found'), 5.0)
        assert not os.path.exists(os.path.join(refused.sites, site))
        refused.close()


def test_a_robot_without_a_map_offers_no_mapping_session(make_rig):
    rig = make_rig(site='gardu', create_site=True, mapping=False)
    assert wait_for(lambda: rig.status.last and rig.status.last.site == 'gardu', 5.0)
    assert rig.call(ListSites, 'site/list').active == 'gardu'  # sites and routes as ever
    with pytest.raises(rospy.ROSException):
        rospy.wait_for_service('mapping/start', 0.5)


def test_site_load_switches_and_resets_localization(make_rig):
    rig = make_rig()
    write_site(rig.sites, 'lobby')
    write_site(rig.sites, 'hall')
    assert rig.call(LoadSite, 'site/load', site='lobby').success
    assert rig.call(LoadSite, 'site/load', site='hall').success
    assert rig.robot.names().count('reset') == 2
    tags = [c for c in rig.robot.calls if c[0] == 'load_tags']
    assert tags[-1][1] == os.path.join(rig.sites, 'hall', 'tags.json') and not tags[-1][2]
    for bad in ('../hall', 'missing', ''):
        assert not rig.call(LoadSite, 'site/load', site=bad).success
    for bad in ('../hall', 'Hall', ''):
        assert not rig.start(bad).success
