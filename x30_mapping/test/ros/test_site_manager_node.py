"""In-process test of site_manager_node against fake localization services."""

from __future__ import annotations

import itertools
import os

import pytest
import rospy
from std_srvs.srv import Trigger

from x30_mapping.site_manager_node import SiteManagerNode
from x30_msgs.msg import LocalizationStatus, SiteStatus
from x30_msgs.srv import ListSites, LoadFile, LoadSite
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, delete_params, wait_for

NAMES = itertools.count()


class FakeRobot:
    """Fake localization services that record their calls."""

    def __init__(self, node, sites_dir):
        self.node, self.sites_dir = node, sites_dir
        self.calls = []
        self.status = node.latched(LocalizationStatus, 'localization/status')
        self.set_status(LocalizationStatus.NONE, '')
        self.services = {
            name: node.create_service(
                srv, name, handler, concurrent=True
            )  # a slow one blocks none
            for srv, name, handler in (
                (LoadFile, 'localization/load_tags', self.load_tags),
                (Trigger, 'localization/reset', self.reset),
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

    def reset(self, _request, response):
        self.calls.append(('reset',))
        self.set_status(LocalizationStatus.NONE, '')
        response.success = True
        return response


def write_site(
    sites_dir,
    site,
    tags='{"base_map": {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}}',
):
    root = os.path.join(sites_dir, site)
    os.makedirs(os.path.join(root, 'routes'), exist_ok=True)
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

    def call(self, srv, name, **request):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, srv)(**request)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.status.close()
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


def test_corrupt_site_files_do_not_crash_the_boot(make_rig):
    """A broken tags file is a note in the status; the node keeps running."""
    rig = make_rig()
    write_site(rig.sites, 'lobby', tags='{broken')
    rig.close()
    rig = make_rig(site='lobby')
    loaded = lambda: rig.status.last and 'loaded lobby' in rig.status.last.message  # noqa: E731
    assert wait_for(loaded, 5.0)
    status = rig.status.last
    assert status.tags == 0 and 'tags.json' in status.message
    assert rig.call(ListSites, 'site/list').active == 'lobby'


def test_the_launch_site_is_created_when_missing(make_rig):
    """create_site: the launch site as an empty folder for routes and tags; never a bad name."""
    rig = make_rig(site='gardu')
    assert wait_for(lambda: rig.status.last and rig.status.last.site == 'gardu', 5.0)
    assert os.path.isdir(os.path.join(rig.sites, 'gardu', 'routes'))
    assert 'loaded gardu: 0 tags, 0 routes' in rig.status.last.message
    assert rig.robot.names().count('reset') == 0  # the origin localization stays
    rig.close()
    for site, create in (('Bad Name', True), ('missing', False)):
        refused = make_rig(site=site, create_site=create)
        assert wait_for(says(refused, 'not found'), 5.0)
        assert not os.path.exists(os.path.join(refused.sites, site))
        refused.close()


def test_no_map_and_no_mapping_session(make_rig):
    rig = make_rig(site='gardu')
    assert wait_for(lambda: rig.status.last and rig.status.last.site == 'gardu', 5.0)
    assert rig.call(ListSites, 'site/list').active == 'gardu'  # sites and routes as ever
    with pytest.raises(rospy.ROSException):
        rospy.wait_for_service('mapping/start', 0.5)
    assert '/map' not in [topic for topic, _ in rospy.get_published_topics()]


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
