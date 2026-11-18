from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_health', 'x30_health.core', 'x30_health.core.checks', 'x30_health.utils'],
        package_dir={'': 'src'},
    )
)
