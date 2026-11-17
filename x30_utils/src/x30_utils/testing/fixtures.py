"""pytest fixtures of ROS tests; a package's test/ros/conftest.py imports what it uses."""

from __future__ import annotations

import pytest

from x30_utils.testing.harness import init_test_node, ros_master


@pytest.fixture(scope='session')
def ros():
    """Run a private rosmaster and init the test process's rospy node, for the session."""
    with ros_master() as uri:
        init_test_node()
        yield uri
