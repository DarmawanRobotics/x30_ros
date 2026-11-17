"""map_server files: PGM reading, trinary conversion, crash-safe pair with backup, cropping."""

from __future__ import annotations

import os

import numpy as np
import pytest
import yaml

from x30_mapping.core import map_io, pgm


def small_grid(value: int = pgm.OCCUPIED, origin=(-1.25, 2.5)) -> map_io.GridMap:
    data = np.full((4, 6), pgm.UNKNOWN, dtype=np.int8)
    data[0, :] = value
    data[1:3, 1:5] = pgm.FREE
    return map_io.GridMap(data, 0.05, *origin)


def test_save_load_round_trip(tmp_path):
    """A saved grid loads back with the same cells, resolution and origin."""
    grid = small_grid()
    yaml_path = map_io.save_map(str(tmp_path / 'site'), grid)
    loaded = map_io.load_map(yaml_path)
    assert np.array_equal(loaded.data, grid.data)
    assert loaded.resolution == pytest.approx(0.05)
    assert (loaded.origin_x, loaded.origin_y) == pytest.approx((-1.25, 2.5))
    assert map_io.load_map(str(tmp_path / 'site')).width == 6  # the directory works too
    assert not [n for n in os.listdir(tmp_path / 'site') if n.startswith('.map-tmp')]


def test_previous_pair_is_kept_as_a_loadable_backup(tmp_path):
    """#5: the second save keeps the first map as map.pgm.bak + map.yaml.bak."""
    site = tmp_path / 'site'
    first, second = small_grid(origin=(0.0, 0.0)), small_grid(origin=(5.0, 5.0))
    map_io.save_map(str(site), first)
    assert not (site / 'map.yaml.bak').exists()
    map_io.save_map(str(site), second)
    backup = map_io.load_map(str(site / 'map.yaml.bak'))
    assert yaml.safe_load((site / 'map.yaml.bak').read_text())['image'] == 'map.pgm.bak'
    assert (backup.origin_x, backup.origin_y) == (0.0, 0.0)
    assert map_io.load_map(str(site)).origin_x == 5.0


def test_torn_pair_is_detected(tmp_path):
    """#5: a new image under an old YAML (crash between the renames) is refused on load."""
    site = tmp_path / 'site'
    map_io.save_map(str(site), small_grid())
    old_yaml = (site / 'map.yaml').read_text()
    bigger = map_io.GridMap(np.zeros((8, 9), np.int8), 0.05, 0.0, 0.0)
    map_io.save_map(str(site), bigger)
    (site / 'map.yaml').write_text(old_yaml)  # the state after a crash between the renames
    with pytest.raises(pgm.MapError, match='torn'):
        map_io.load_map(str(site))
    same_size = small_grid(value=pgm.FREE)
    map_io.save_map(str(site), small_grid())
    old_yaml = (site / 'map.yaml').read_text()
    map_io.save_map(str(site), same_size)
    (site / 'map.yaml').write_text(old_yaml)
    with pytest.raises(pgm.MapError, match='CRC'):
        map_io.load_map(str(site))


def test_failed_write_leaves_the_old_pair(tmp_path, monkeypatch):
    """#5: an error while writing keeps the old map intact and leaves no temporary files."""
    site = tmp_path / 'site'
    map_io.save_map(str(site), small_grid())
    before = (site / 'map.pgm').read_bytes(), (site / 'map.yaml').read_text()

    def broken(path, data):
        raise OSError('disk full')

    monkeypatch.setattr(map_io, '_write_synced', broken)
    with pytest.raises(OSError):
        map_io.save_map(str(site), small_grid(origin=(9.0, 9.0)))
    assert ((site / 'map.pgm').read_bytes(), (site / 'map.yaml').read_text()) == before
    assert not [n for n in os.listdir(site) if n.startswith('.map-tmp')]


def test_image_row_zero_is_the_top(tmp_path):
    """Row 0 of the image is the largest y: the bottom grid row comes last in the file."""
    data = np.full((3, 2), pgm.FREE, dtype=np.int8)
    data[0, 0] = pgm.OCCUPIED  # bottom-left cell
    map_io.save_map(str(tmp_path), map_io.GridMap(data, 0.1, 0.0, 0.0))
    image = pgm.read_image(str(tmp_path / 'map.pgm'))
    assert image[-1, 0] == pgm.PIXEL_OCCUPIED
    assert image[0, 0] == pgm.PIXEL_FREE


def test_reads_map_server_files_with_comments_and_ascii(tmp_path):
    """A map_saver P5 with a comment and an ASCII P2 both read, thresholds as map_server."""
    p5 = tmp_path / 'a.pgm'
    pixels = np.array([[0, 254, 205]], dtype=np.uint8)
    p5.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n3 1\n255\n' + pixels.tobytes())
    assert pgm.read_image(str(p5)).tolist() == [[0, 254, 205]]
    p2 = tmp_path / 'b.pgm'
    p2.write_text('P2\n3 1\n# a comment\n255\n0 254\n205\n')
    assert pgm.read_image(str(p2)).tolist() == [[0, 254, 205]]
    (tmp_path / 'map.yaml').write_text(
        'image: a.pgm\nresolution: 0.05\norigin: [-10.0, -5.0, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n'
    )
    grid = map_io.load_map(str(tmp_path / 'map.yaml'))
    assert grid.data.tolist() == [[100, 0, -1]]
    assert grid.origin_x == -10.0


def test_sixteen_bit_and_negate(tmp_path):
    """16-bit PGMs are scaled to 8 bits; negate flips black and white."""
    path = tmp_path / 'c.pgm'
    path.write_bytes(b'P5 2 1 65535 ' + np.array([0, 65535], dtype='>u2').tobytes())
    assert pgm.read_image(str(path)).tolist() == [[0, 255]]
    image = np.array([[0, 255]], dtype=np.uint8)
    assert pgm.image_to_occupancy(image, negate=True).tolist() == [[0, 100]]


def test_broken_files_raise_map_error(tmp_path):
    """Missing, truncated or foreign files raise MapError, never anything else."""
    with pytest.raises(pgm.MapError):
        map_io.load_map(str(tmp_path / 'missing.yaml'))
    bad = tmp_path / 'bad.pgm'
    bad.write_bytes(b'P5\n10 10\n255\n' + b'\x00' * 5)
    with pytest.raises(pgm.MapError):
        pgm.read_image(str(bad))
    with pytest.raises(pgm.MapError):
        pgm.read_image(str(tmp_path / 'missing.pgm'))
    foreign = tmp_path / 'f.pgm'
    foreign.write_bytes(b'GIF89a')
    with pytest.raises(pgm.MapError):
        pgm.read_image(str(foreign))
    (tmp_path / 'list.yaml').write_text('- 1\n- 2\n')
    with pytest.raises(pgm.MapError):
        map_io.load_map(str(tmp_path / 'list.yaml'))
    (tmp_path / 'size.yaml').write_text('image: a.pgm\nresolution: 0.1\nimage_size: oops\n')
    with pytest.raises(pgm.MapError):
        map_io.load_map(str(tmp_path / 'size.yaml'))


def test_crop_known_keeps_cells_in_place():
    """Cropping to the known cells shifts the origin so every cell keeps its position."""
    data = np.full((100, 100), pgm.UNKNOWN, dtype=np.int8)
    data[40:45, 60:62] = pgm.FREE
    grid = map_io.GridMap(data, 0.1, -5.0, -5.0)
    cropped = map_io.crop_known(grid, 0.2)
    assert cropped.data.shape == (5 + 4, 2 + 4)
    assert cropped.origin_x == pytest.approx(-5.0 + 58 * 0.1)
    assert cropped.origin_y == pytest.approx(-5.0 + 38 * 0.1)
    empty = map_io.GridMap(np.full((3, 3), -1, np.int8), 1, 0, 0)
    assert map_io.crop_known(empty, 1).width == 3
