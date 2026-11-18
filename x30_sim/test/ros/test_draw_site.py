"""draw_site: a site from a world once (kept after), redrawn on request, bad input exits 2."""

from __future__ import annotations

from pathlib import Path

from x30_sim.draw_site import INVALID, draw, main
from x30_utils.io import SiteLayout, read_route, read_tags


def test_draws_once_then_keeps_the_map(tmp_path, capsys):
    assert main(['--site', 'lobby', '--sites-dir', str(tmp_path)]) == 0
    layout = SiteLayout(str(tmp_path), 'lobby')
    assert capsys.readouterr().out.strip() == layout.root
    assert Path(layout.map_yaml).is_file() and Path(layout.map_image).is_file()
    assert 'charging_dock' in read_tags(layout.tags)[0]
    assert read_route(str(Path(layout.routes) / 'mall_loop.csv'))
    Path(layout.map_image).write_bytes(b'changed')
    assert draw('', 'lobby', str(tmp_path)) == (layout.root, False)
    assert Path(layout.map_image).read_bytes() == b'changed'
    assert draw('mall_loop', 'lobby', str(tmp_path), overwrite=True) == (layout.root, True)
    assert Path(layout.map_image).read_bytes().startswith(b'P5')


def test_bad_names_and_worlds_exit_2(tmp_path, capsys):
    assert main(['--site', '../up', '--sites-dir', str(tmp_path)]) == INVALID
    assert main(['--site', 'ok', '--world', 'atlantis', '--sites-dir', str(tmp_path)]) == INVALID
    assert 'atlantis' in capsys.readouterr().err
    assert not (tmp_path / 'ok').exists()
