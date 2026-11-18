"""Where records go: the console (loguru), JSON lines files (rotated) and the ROS logger."""

from __future__ import annotations

import contextlib
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
import sys

from x30_utils.log.logger import LEVEL_NO, Record

try:
    from loguru import logger as _loguru
except ImportError:  # pragma: no cover - loguru is optional
    _loguru = None

LOGURU_FORMAT = (
    '<green>{time:HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | '
    '<cyan>{extra[logger]}</cyan> | <level>{message}</level>'
)
_LOGURU_LEVEL = {'warning': 'WARNING', 'critical': 'CRITICAL'}


class Sink:
    """Base sink with a minimum level."""

    def __init__(self, level: str = 'debug'):
        self.level = level

    def emit(self, record: Record):
        """Write the record when its level is enabled."""
        if LEVEL_NO[record.level] >= LEVEL_NO[self.level]:
            self.write(record)

    def write(self, record: Record):
        """Write one record."""
        raise NotImplementedError


class ConsoleSink(Sink):
    """Colored console output through loguru, plain stderr without it."""

    _installed = False

    def __init__(self, level: str = 'info'):
        super().__init__(level)
        if _loguru is not None and not ConsoleSink._installed:
            with contextlib.suppress(ValueError):
                _loguru.remove(0)
            _loguru.add(
                sys.stderr,
                format=LOGURU_FORMAT,
                level='DEBUG',
                filter=lambda r: r['extra'].get('x30', False),
            )
            ConsoleSink._installed = True

    def write(self, record: Record):
        """Print the record."""
        if _loguru is not None:
            level = _LOGURU_LEVEL.get(record.level, record.level.upper())
            _loguru.bind(x30=True, logger=record.logger).log(level, record.text())
        else:
            stamp = datetime.fromtimestamp(record.wall_time).strftime('%H:%M:%S.%f')[:-3]
            print(
                f'{stamp} | {record.level.upper():<8} | {record.logger} | {record.text()}',
                file=sys.stderr,
            )


class JsonlSink(Sink):
    """One JSON object per line, rotated by size (loguru: compressed); read after an incident."""

    def __init__(self, path: str, level: str = 'debug', max_mb: float = 10.0, keep: int = 5):
        super().__init__(level)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._handler = None
        self._handler_id = None
        if _loguru is not None:
            self._handler_id = _loguru.add(
                path,
                format='{message}',
                level='DEBUG',
                rotation=f'{max_mb} MB',
                retention=keep,
                compression='gz',
                enqueue=True,
                filter=lambda r: r['extra'].get('x30_file') == path,
            )
        else:
            self._handler = RotatingFileHandler(
                path, maxBytes=int(max_mb * 1e6), backupCount=keep, encoding='utf-8'
            )
            self._handler.setFormatter(logging.Formatter('%(message)s'))

    def write(self, record: Record):
        """Append the record as JSON."""
        line = json.dumps(
            {
                'time': datetime.fromtimestamp(record.wall_time, timezone.utc).isoformat(),
                'level': record.level,
                'logger': record.logger,
                'message': record.message,
                **{k: _jsonable(v) for k, v in record.fields.items()},
            }
        )
        if self._handler is not None:
            self._handler.emit(logging.makeLogRecord({'msg': line, 'args': None}))
        else:
            _loguru.bind(x30_file=self.path).info(line)

    def close(self):
        """Close the file."""
        if self._handler is not None:
            self._handler.close()
        elif self._handler_id is not None:
            _loguru.remove(self._handler_id)
            self._handler_id = None


class RosSink(Sink):
    """Forward to a ROS logger (debug, info, warning, error, fatal), so records reach /rosout."""

    def __init__(self, ros_logger, level: str = 'debug'):
        super().__init__(level)
        self.ros_logger = ros_logger
        self._writers = {
            'debug': self._debug,
            'info': self._info,
            'warning': self._warning,
            'error': self._error,
            'critical': self._fatal,
        }

    def write(self, record: Record):
        """Log through ROS; a logging failure never propagates into the caller."""
        try:
            self._writers[record.level](record.text())
        except Exception as error:  # a broken sink must not take the node down
            print(f'x30 log sink failed: {error}: {record.text()}', file=sys.stderr)

    def _debug(self, text: str):
        self.ros_logger.debug(text)

    def _info(self, text: str):
        self.ros_logger.info(text)

    def _warning(self, text: str):
        self.ros_logger.warning(text)

    def _error(self, text: str):
        self.ros_logger.error(text)

    def _fatal(self, text: str):
        self.ros_logger.fatal(text)


def _jsonable(value):
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    return str(value)
