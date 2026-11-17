"""pytest fixtures of ROS tests; a package's test/ros/conftest.py imports what it uses."""

from __future__ import annotations

import pytest

from x30_utils.testing.harness import init_test_node, ros_master

# one master and one rospy node per process: pytest makes a fixture of every conftest importing
# ros, and a second master would leave the node registered with the first (one pytest run of
# several packages' tests)
_SESSION: dict[str, str] = {}


@pytest.fixture(scope='session')
def ros():
    """Run a private rosmaster and init the test process's rospy node, for the session."""
    if 'uri' in _SESSION:
        yield _SESSION['uri']
        return
    with ros_master() as uri:
        init_test_node()
        _SESSION['uri'] = uri
        try:
            yield uri
        finally:
            _SESSION.clear()
