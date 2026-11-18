"""Structured logging: get_logger(name).info(msg, key=value); X30_LOG_LEVEL, X30_LOG_DIR."""

from __future__ import annotations

import os

from x30_utils.log.logger import LEVELS, Logger, Record
from x30_utils.log.sinks import ConsoleSink, JsonlSink, RosSink, Sink

_files: dict[str, JsonlSink] = {}


def _level() -> str:
    level = os.environ.get('X30_LOG_LEVEL', 'info').lower()
    return level if level in LEVELS else 'info'


def file_sinks(name: str) -> list[Sink]:
    """Return the JSON lines sink for name when X30_LOG_DIR is set."""
    directory = os.environ.get('X30_LOG_DIR', '')
    if not directory:
        return []
    stem = name.strip('/').replace('/', '.')
    path = os.path.join(os.path.expanduser(directory), f'{stem}.jsonl')
    if path not in _files:
        _files[path] = JsonlSink(path)
    return [_files[path]]


def get_logger(name: str) -> Logger:
    """Return a logger printing to the console (and to a file with X30_LOG_DIR)."""
    return Logger(name, [ConsoleSink(_level()), *file_sinks(name)])


def ros_logger(name: str, logger) -> Logger:
    """Return a logger for a node: rospy (console and /rosout), plus the optional file."""
    return Logger(name, [RosSink(logger), *file_sinks(name)])


__all__ = [
    'ConsoleSink',
    'JsonlSink',
    'Logger',
    'Record',
    'RosSink',
    'Sink',
    'file_sinks',
    'get_logger',
    'ros_logger',
]
