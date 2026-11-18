from __future__ import annotations

from pathlib import Path

from x30_utils.testing import check_package


def test_package_conventions():
    problems = check_package(Path(__file__).resolve().parents[1])
    assert not problems, '\n'.join(problems)
