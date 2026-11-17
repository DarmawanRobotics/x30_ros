from __future__ import annotations

import ast
import textwrap

from x30_utils.testing import check_package
from x30_utils.testing.py38 import py38_problems

SAMPLE = textwrap.dedent(
    """
    from __future__ import annotations
    from itertools import pairwise
    import functools
    from dataclasses import dataclass

    Alias = tuple[int, ...]


    def f(a: list[int], b: dict[str, int] | None = None) -> tuple[int, ...]:
        x: list[int] = []
        isinstance(a, int | float)
        zip(a, a, strict=True)
        'abc'.removeprefix('a')
        functools.cache(f)
        flags = 1 | 2
        return tuple(x), flags


    @dataclass(slots=True)
    class C:
        names: tuple[str, ...] = ()


    'abc'.removesuffix('c')  # py38: ok
    import zoneinfo
    from importlib.resources import files
    Path('a').is_relative_to('b')
    """
)


def _lines(problems):
    return sorted(int(p.split(':')[1].split()[0]) for p in problems)


def test_runtime_py39_features_are_reported():
    problems = py38_problems(SAMPLE, ast.parse(SAMPLE), 'x.py')
    assert _lines(problems) == [3, 7, 12, 13, 14, 15, 20, 26, 27, 28], '\n'.join(problems)
    text = '\n'.join(problems)
    hints = (
        'zip(xs, xs[1:])', 'typing.Tuple', 'isinstance(x, (A, B))', 'length check',
        'removeprefix', 'lru_cache', 'a plain dataclass', 'datetime.timezone',
        'rospkg', 'resolve_inside',
    )  # fmt: skip
    for hint in hints:
        assert hint in text, hint
    assert all(p.endswith('(Python 3.8 / Noetic)') for p in problems)


def test_annotations_are_fine():
    clean = textwrap.dedent(
        """
        from __future__ import annotations
        from typing import Tuple

        Alias = Tuple[int, ...]


        def f(a: list[int] | None, *rest: tuple[str, ...], **kw: dict[str, int]) -> set[int]:
            names: frozenset[str] = frozenset()
            return {len(names)}
        """
    )
    assert py38_problems(clean, ast.parse(clean), 'y.py') == []


def test_py38_checks_follow_the_ruff_target(tmp_path):
    (tmp_path / 'pkg').mkdir()
    (tmp_path / 'pkg' / 'mod.py').write_text('"""Doc."""\n\nX = list[int]\n')
    (tmp_path / 'ruff.toml').write_text('target-version = "py310"\n')
    assert not check_package(tmp_path / 'pkg')
    (tmp_path / 'ruff.toml').write_text('target-version = "py38"\n')
    assert check_package(tmp_path / 'pkg')
    assert not check_package(tmp_path / 'pkg', py38=False)


def test_core_must_not_import_ros(tmp_path):
    core = tmp_path / 'pkg' / 'src' / 'pkg' / 'core'
    core.mkdir(parents=True)
    (core / 'algo.py').write_text(
        '"""Doc."""\n\nimport rospy\nfrom x30_msgs.msg import Waypoint\n'
    )
    problems = check_package(tmp_path / 'pkg', py38=False)
    assert [p.split(' ', 1)[1] for p in problems] == [
        'core/ imports rospy',
        'core/ imports x30_msgs.msg',
    ]


def test_docstrings_are_one_line(tmp_path):
    (tmp_path / 'pkg').mkdir()
    module = tmp_path / 'pkg' / 'mod.py'
    module.write_text('"""Doc."""\n\n\ndef f():\n    """Return 1."""\n    return 1\n')
    assert not check_package(tmp_path / 'pkg', py38=False)
    module.write_text('"""Doc."""\n\n\ndef f():\n    """\n    Return 1.\n\n    More.\n    """\n')
    assert check_package(tmp_path / 'pkg', py38=False) == [
        'mod.py:5 docstring of more than one line, shorten it'
    ]
