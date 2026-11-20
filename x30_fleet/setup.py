from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_fleet', 'x30_fleet.core', 'x30_fleet.utils', 'x30_fleet.tools'],
        package_dir={'': 'src'},
    )
)
