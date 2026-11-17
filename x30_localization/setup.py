from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=[
            'x30_localization',
            'x30_localization.core',
            'x30_localization.core.sources',
            'x30_localization.utils',
        ],
        package_dir={'': 'src'},
    )
)
