"""Configuration of teleop_udp_node (limits and frames from the robot profile, robot/*)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_bridge.core.protocol import PORT
from x30_utils.config import param

# the X30 driver's Trigger services the handheld offers as actions (x30_driver README)
ROBOT_COMMANDS = ('stand_up', 'sit_down', 'start_stepping', 'stop_stepping', 'gait_walk',
                  'gait_off_road', 'gait_stair', 'emergency_stop')  # fmt: skip
CAMERAS = ('visible=/hikmicro/visible/image_raw/compressed',
           'thermal=/hikmicro/thermal/image_raw/compressed')  # fmt: skip


@dataclass(frozen=True)
class TeleopConfig:
    """teleop_udp_node: the handheld's UDP gateway."""

    port: int = param(PORT, min=1, max=65535, live=False, doc='UDP port')
    token_env: str = param(
        'X30_TELEOP_TOKEN', live=False, doc='environment variable of the shared secret'
    )
    robot_name: str = param('', live=False, doc='name in discovery beacons ("": hostname)')
    client_timeout_sec: float = param(
        2.0, min=0.2, max=60.0, unit='s', live=False, doc='a silent handheld is dropped'
    )
    max_vx: float = param(0.8, min=0.0, max=5.0, unit='m/s', live=False, doc='stick limit')
    max_vy: float = param(0.4, min=0.0, max=5.0, unit='m/s', live=False, doc='stick limit')
    max_wz: float = param(1.0, min=0.0, max=10.0, unit='rad/s', live=False, doc='stick limit')
    remote_source: str = param(
        'handheld', live=False, doc='control mux source: control/remote/<source>/cmd_vel'
    )
    telemetry_hz: float = param(10.0, min=0.5, max=50.0, unit='Hz', live=False, doc='state rate')
    robot_command_prefix: str = param(
        '/x30/', live=False, doc="the driver's Trigger services (bringup's namespace)"
    )
    robot_commands: tuple[str, ...] = param(
        ROBOT_COMMANDS, live=False, doc="the handheld's action menu, std_srvs/Trigger each"
    )
    battery_topic: str = param('/x30/battery', live=False, doc="the driver's BatteryState")
    cmd_topic: str = param('/x30/cmd_vel', live=False, doc="the mux's command to the driver")
    driver_diagnostics_topic: str = param(
        '/x30/diagnostics', live=False, doc="the driver's status line"
    )
    health_diagnostics_topic: str = param(
        '/diagnostics', live=False, doc="x30_health's findings (the computer's figures)"
    )
    sites_dir: str = param('~/x30/map', live=False, doc='taught routes are listed from here')
    cameras: tuple[str, ...] = param(
        CAMERAS, live=False, doc='name=topic of JPEG frames (CompressedImage); the first is shown'
    )
    camera_fps: float = param(5.0, min=0.5, max=30.0, unit='Hz', live=False, doc='frame cap')
    camera_jpeg_quality: int = param(60, min=10, max=100, live=False, doc='re-encoded frames')
    camera_max_width: int = param(640, min=64, max=4096, live=False, doc='re-encoded frames')
    camera_max_bytes: int = param(
        60000, min=1000, max=10000000, live=False, doc='bigger JPEGs are re-encoded'
    )
    lidar_topic: str = param(
        '/cloud_registered_body', live=False, doc="FAST-LIO's scan in body ('': no cloud)"
    )
    lidar_hz: float = param(2.0, min=0.1, max=20.0, unit='Hz', live=False, doc='cloud rate')
    lidar_voxel: float = param(0.1, min=0.01, max=2.0, unit='m', live=False, doc='downsampling')
    lidar_max_points: int = param(3000, min=10, max=100000, live=False, doc='points per cloud')
    navigation_node: str = param(
        '/path_follower_node', live=False, doc='its speed/cruise is the patrol speed'
    )
    patrol_speed_min: float = param(
        0.1, min=0.01, max=5.0, unit='m/s', live=False, doc='slowest patrol speed to set'
    )
    patrol_speed_max: float = param(
        0.6, min=0.01, max=5.0, unit='m/s', live=False, doc='fastest (and robot limits/max_vx)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.patrol_speed_min > self.patrol_speed_max:
            problems.append('patrol_speed_min must not exceed patrol_speed_max')
        for item in self.cameras:
            name, sep, topic = item.partition('=')
            if not (name and sep and topic.startswith('/')):
                problems.append(f'cameras: {item!r} is not name=/topic')
        return problems
