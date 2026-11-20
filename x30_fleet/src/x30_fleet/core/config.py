"""Configuration of fleet_uplink_node (frames from the robot profile, robot/*)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import param


@dataclass(frozen=True)
class FleetConfig:
    """fleet_uplink_node: the robot side of the NETRA command center."""

    server_url: str = param(
        '', live=False, doc='NETRA, e.g. https://netra.example.com ("": spool only)'
    )
    token_env: str = param(
        'X30_NETRA_TOKEN', live=False, doc="environment variable of the robot's NETRA token"
    )
    robot_id: str = param('x30-01', live=False, doc='fleet id of this robot in NETRA')
    verify_tls: bool = param(True, live=False, doc='check the server certificate')
    telemetry_hz: float = param(1.0, min=0.1, max=10.0, unit='Hz', live=False, doc='snapshots')
    upload_period_sec: float = param(
        5.0, min=0.5, max=600.0, unit='s', live=False, doc='spooled uploads are tried this often'
    )
    spool_dir: str = param(
        '~/x30/spool', live=False, doc='routes and inspections wait here until NETRA has them'
    )
    sites_dir: str = param(
        '~/x30/map', live=False, doc="NETRA's routes and plans go into the active site here"
    )
    clock_sync_period_sec: float = param(
        10.0, min=1.0, max=3600.0, unit='s', live=False, doc="measure the offset to NETRA's clock"
    )
    clock_warn_ms: float = param(
        200.0, min=0.0, max=1e7, unit='ms', doc="warn when the robot's clock differs this much"
    )
    battery_topic: str = param('/x30/battery', live=False, doc="the driver's BatteryState")
    health_diagnostics_topic: str = param(
        '/diagnostics', live=False, doc="x30_health's findings (the computer's figures)"
    )
    driver_diagnostics_topic: str = param(
        '/x30/diagnostics', live=False, doc="the driver's status line"
    )
