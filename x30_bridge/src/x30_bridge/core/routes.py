"""Taught routes under the sites directory, as the handheld lists and loads them."""

from __future__ import annotations

import os

from x30_utils.io import PathError, expand, resolve_inside

MAX_ROUTES = 50


def list_routes(sites_dir: str) -> list[str]:
    """Return route CSVs inside sites_dir, newest first, relative to it (<site>/routes/x.csv)."""
    root = os.path.realpath(expand(sites_dir))
    found = []
    for directory, _, files in os.walk(root):
        for name in files:
            if name.endswith('.csv'):
                path = os.path.join(directory, name)
                try:
                    found.append((os.path.getmtime(path), os.path.relpath(path, root)))
                except OSError:  # deleted while listing
                    continue
    return [path for _, path in sorted(found, reverse=True)[:MAX_ROUTES]]


def route_path(sites_dir: str, file) -> str:
    """Return a route from the handheld as a path inside sites_dir; ValueError otherwise."""
    text = str(file or '').strip()
    if not text.endswith('.csv'):
        raise ValueError('route must be a .csv file of a site')
    root = os.path.realpath(expand(sites_dir))
    try:
        full = resolve_inside(root, text)
    except PathError as error:
        raise ValueError(f'route must be a file under {sites_dir}') from error
    return os.path.relpath(full, root)
