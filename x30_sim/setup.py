from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_sim', 'x30_sim.core', 'x30_sim.utils'],
        package_dir={'': 'src'},
    )
)
