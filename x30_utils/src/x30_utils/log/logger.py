"""Structured logging: log.info('scan dropped', topic=t, age_ms=120), with throttle and once."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
import traceback

LEVELS = ('debug', 'info', 'warning', 'error', 'critical')
LEVEL_NO = {name: i for i, name in enumerate(LEVELS)}


@dataclass
class Record:
    """One log entry."""

    level: str
    logger: str
    message: str
    fields: dict = field(default_factory=dict)
    wall_time: float = field(default_factory=time.time)

    def text(self) -> str:
        """Return the message followed by its fields as key=value."""
        if not self.fields:
            return self.message
        pairs = ' '.join(f'{k}={_format(v)}' for k, v in self.fields.items())
        return f'{self.message} {pairs}'


class Logger:
    """Send records to sinks (console, JSON lines file, ROS)."""

    def __init__(self, name: str, sinks: list, context: dict | None = None):
        self.name = name
        self.sinks = list(sinks)
        self.context = dict(context or {})
        self._last: dict = {}
        self._once: set = set()
        self._lock = threading.Lock()

    def bind(self, **context) -> Logger:
        """Return a logger that adds context fields to every record."""
        return Logger(self.name, self.sinks, {**self.context, **context})

    def debug(self, message: str, /, **kwargs):
        """Log at debug level."""
        self.log('debug', message, **kwargs)

    def info(self, message: str, /, **kwargs):
        """Log at info level."""
        self.log('info', message, **kwargs)

    def warning(self, message: str, /, **kwargs):
        """Log at warning level."""
        self.log('warning', message, **kwargs)

    warn = warning

    def error(self, message: str, /, **kwargs):
        """Log at error level."""
        self.log('error', message, **kwargs)

    def critical(self, message: str, /, **kwargs):
        """Log at critical level."""
        self.log('critical', message, **kwargs)

    def exception(self, message: str, /, **kwargs):
        """Log an error with the traceback of the exception being handled."""
        self.log('error', message, exception=traceback.format_exc().strip(), **kwargs)

    def event(self, name: str, /, **fields):
        """Log a named event (state changes worth auditing)."""
        self.log('info', name, event=name, **fields)

    def log(
        self,
        level: str,
        message: str,
        /,
        *,
        throttle: float | None = None,
        once: bool = False,
        key: str | None = None,
        **fields,
    ):
        """Log message; throttle: seconds between repeats; once: only the first time."""
        if not self._allowed(key or f'{level}:{message}', throttle, once):
            return
        record = Record(level, self.name, message, {**self.context, **fields})
        for sink in self.sinks:
            sink.emit(record)

    def _allowed(self, key: str, throttle: float | None, once: bool) -> bool:
        with self._lock:
            if once:
                if key in self._once:
                    return False
                self._once.add(key)
            if throttle:
                now = time.monotonic()
                last = self._last.get(key)
                if last is not None and now - last < throttle:
                    return False
                self._last[key] = now
        return True


def _format(value) -> str:
    if isinstance(value, float):
        return 'inf' if math.isinf(value) else f'{value:.4g}'
    if isinstance(value, str) and (' ' in value or not value):
        return repr(value)
    return str(value)
