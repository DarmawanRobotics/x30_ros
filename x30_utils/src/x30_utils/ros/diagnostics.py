from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


def status(
    name: str, level: str, message: str, hardware_id: str = '', **values
) -> DiagnosticStatus:
    """Return a DiagnosticStatus with key/value pairs."""
    return DiagnosticStatus(
        level=LEVELS[level],
        name=name,
        message=message,
        hardware_id=hardware_id,
        values=[KeyValue(key=k, value=_text(v)) for k, v in values.items()],
    )


def _text(value) -> str:
    return f'{value:.3g}' if isinstance(value, float) else str(value)
