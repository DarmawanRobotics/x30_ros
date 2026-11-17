"""PGM images (P5 binary, P2 ASCII) and their map_server occupancy interpretation."""

from __future__ import annotations

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
PIXEL_OCCUPIED, PIXEL_FREE, PIXEL_UNKNOWN = 0, 254, 205
MODES = ('trinary', 'scale', 'raw')


class MapError(ValueError):
    """A map file is missing, broken or not understood."""


def _tokens(stream: bytes, count: int, start: int) -> tuple[list[bytes], int]:
    """Read count whitespace-separated header tokens, skipping # comments."""
    tokens, i, n = [], start, len(stream)
    while len(tokens) < count:
        while i < n and stream[i : i + 1].isspace():
            i += 1
        if i < n and stream[i : i + 1] == b'#':
            while i < n and stream[i : i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < n and not stream[j : j + 1].isspace() and stream[j : j + 1] != b'#':
            j += 1
        if j == i:
            raise MapError('truncated PGM header')
        tokens.append(stream[i:j])
        i = j
    return tokens, i


def decode_pgm(raw: bytes, name: str = 'image') -> np.ndarray:
    """Decode a P5 or P2 PGM to uint8 rows top first (16-bit scaled to 8)."""
    magic = raw[:2]
    if magic not in (b'P5', b'P2'):
        raise MapError(f'{name} is not a PGM (magic {magic!r})')
    (width, height, maxval), end = _tokens(raw, 3, 2)
    try:
        width, height, maxval = int(width), int(height), int(maxval)
    except ValueError as error:
        raise MapError(f'{name}: bad PGM header') from error
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise MapError(f'{name}: bad PGM size {width}x{height} maxval {maxval}')
    count = width * height
    if magic == b'P5':
        body = raw[end + 1 :]  # one whitespace byte after maxval
        dtype = np.dtype('>u2') if maxval > 255 else np.dtype('u1')
        expected = count * dtype.itemsize
        if len(body) < expected:
            raise MapError(f'{name}: {len(body)} bytes of pixels, expected {expected}')
        values = np.frombuffer(body, dtype=dtype, count=count).astype(np.float64)
    else:
        try:
            values = np.array(raw[end:].split()[:count], dtype=np.float64)
        except ValueError as error:
            raise MapError(f'{name}: bad ASCII pixels') from error
        if values.size < count:
            raise MapError(f'{name}: {values.size} pixels, expected {count}')
    if maxval != 255:
        values = np.round(values * 255.0 / maxval)
    return np.clip(values, 0, 255).astype(np.uint8).reshape(height, width)


def encode_pgm(image: np.ndarray, comment: str = '') -> bytes:
    """Encode uint8 rows top first as a binary PGM (P5)."""
    image = np.ascontiguousarray(image, dtype=np.uint8)
    height, width = image.shape
    header = b'P5\n'
    if comment:
        header += f'# {comment}\n'.encode()
    header += f'{width} {height}\n255\n'.encode()
    return header + image.tobytes()


def read_image(path: str) -> np.ndarray:
    """Read a map image as uint8 grey rows top first: PGM natively, others with OpenCV."""
    if not path.lower().endswith(('.pgm', '.pgm.bak')):
        return _read_other(path)
    try:
        with open(path, 'rb') as f:
            raw = f.read()
    except OSError as error:
        raise MapError(f'cannot read {path}: {error}') from error
    return decode_pgm(raw, path)


def _read_other(path: str) -> np.ndarray:
    try:
        import cv2
    except ImportError as error:
        raise MapError(f'{path}: only PGM maps can be read without OpenCV') from error
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise MapError(f'cannot read image {path}')
    return image


def image_to_occupancy(
    image: np.ndarray,
    negate: bool = False,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.25,
    mode: str = 'trinary',
) -> np.ndarray:
    """Convert an image (rows top first) to OccupancyGrid values (row 0 at the bottom)."""
    if mode not in MODES:
        raise MapError(f'unknown map mode {mode!r}, use trinary, scale or raw')
    shade = image.astype(np.float64)
    if mode == 'raw':
        data = np.where(shade > 100.0, UNKNOWN, shade)
    else:
        p = shade / 255.0 if negate else (255.0 - shade) / 255.0
        data = np.full(p.shape, float(UNKNOWN))
        if mode == 'scale':
            span = max(occupied_thresh - free_thresh, 1e-9)
            data = np.round(99.0 * (p - free_thresh) / span)
        data[p > occupied_thresh] = OCCUPIED
        data[p < free_thresh] = FREE
    return np.flipud(data).astype(np.int8)


def occupancy_to_image(
    data: np.ndarray, occupied_thresh: float = 0.65, free_thresh: float = 0.25
) -> np.ndarray:
    """Convert OccupancyGrid values (row 0 at the bottom) to a trinary image (rows top first)."""
    data = np.asarray(data, dtype=np.int16)
    image = np.full(data.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    known = data >= 0
    image[known & (data >= occupied_thresh * 100.0)] = PIXEL_OCCUPIED
    image[known & (data <= free_thresh * 100.0)] = PIXEL_FREE
    return np.flipud(image)
