from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=[
            'x30_navigation',
            'x30_navigation.core',
            'x30_navigation.core.avoidance',
            'x30_navigation.core.config',
            'x30_navigation.core.controllers',
            'x30_navigation.core.docking',
            'x30_navigation.core.follower',
            'x30_navigation.core.path',
            'x30_navigation.core.shift',
            'x30_navigation.core.sites',
            'x30_navigation.utils',
        ],
        package_dir={'': 'src'},
    )
)
