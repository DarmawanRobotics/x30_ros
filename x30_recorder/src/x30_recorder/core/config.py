"""Configuration of recorder_node: what the perception host records, when and within what disk."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param


@dataclass(frozen=True)
class ThermalConfig:
    """Raw thermal frames of the HIKMICRO camera (hikmicro_driver's radiometric image)."""

    enabled: bool = param(True, live=False, doc='raw thermal frames (.npz) while recording')
    topic: str = param(
        '/hikmicro/thermal/temperature', live=False, doc='32FC1, degC per pixel (hikmicro_driver)'
    )
    ptz_topic: str = param('/hikmicro/ptz/state', live=False, doc='pan, tilt, zoom of each frame')
    min_interval_sec: float = param(
        0.0, min=0.0, max=3600.0, unit='s', doc='at most a frame this often; 0: every frame'
    )


@dataclass(frozen=True)
class VideoConfig:
    """The camera streams, copied as they come (no decoding, no encoding) into MP4 segments."""

    enabled: bool = param(True, live=False, doc='the streams (.mp4) while recording')
    host: str = param('192.168.1.64', live=False, doc='the HIKMICRO camera')
    port: int = param(554, min=1, max=65535, live=False, doc='its RTSP port')
    user: str = param('admin', live=False, doc='its user')
    password_env: str = param(
        'HIKMICRO_PASSWORD', live=False, doc='environment variable with its password (no param)'
    )
    streams: tuple[str, ...] = param(
        ('visible:101', 'thermal:201'),
        live=False,
        doc='name:channel of rtsp://<host>/Streaming/Channels/<channel> (101, 201: main streams)',
    )
    urls: tuple[str, ...] = param(
        (), live=False, doc='more streams as name=url (any ffmpeg input), recorded as they are'
    )
    transport: str = param('tcp', choices=('tcp', 'udp'), live=False, doc='RTSP transport')
    segment_sec: float = param(
        300.0, min=10.0, max=3600.0, unit='s', live=False, doc='MP4 per segment, at a keyframe'
    )
    restart_sec: float = param(
        2.0, min=0.5, max=60.0, unit='s', live=False, doc='a stopped stream restarts, up to 60 s'
    )
    ffmpeg: str = param('ffmpeg', live=False, doc='the ffmpeg binary')


@dataclass(frozen=True)
class AcousticConfig:
    """The Hertzino HZ-FA-371's data (hertzino_driver): spectra, raw frames, detections."""

    enabled: bool = param(True, live=False, doc='the acoustic camera data while recording')
    namespace: str = param('/hertzino', live=False, doc="hertzino_driver's topics")
    chunk_sec: float = param(
        10.0, min=1.0, max=600.0, unit='s', live=False, doc='a .npz of each stream this often'
    )


@dataclass(frozen=True)
class PatrolConfig:
    """Recording on its own while the path follower runs a route."""

    enabled: bool = param(True, live=False, doc='record while a route runs (AUTO, patrol)')
    status_topic: str = param('/navigation/status', live=False, doc="x30_navigation's status")
    stop_after_sec: float = param(
        30.0, min=0.0, max=3600.0, unit='s', doc='no route this long: stop (laps stay one session)'
    )
    silence_sec: float = param(
        60.0, min=1.0, max=3600.0, unit='s', doc='no navigation status this long: no route runs'
    )


@dataclass(frozen=True)
class DiskConfig:
    """The recordings within the perception host's disk, which Deep Robotics' software shares."""

    keep_days: int = param(0, min=0, max=3650, doc='older sessions go; 0: kept while room is left')
    min_free_gb: float = param(
        10.0, min=0.0, max=10000.0, unit='GB', doc='the oldest sessions go while less is free'
    )
    stop_free_gb: float = param(
        3.0, min=0.0, max=10000.0, unit='GB', doc='below this nothing more is written'
    )
    check_sec: float = param(
        30.0, min=1.0, max=3600.0, unit='s', doc='disk checked this often while recording'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.stop_free_gb > self.min_free_gb:
            return ['stop_free_gb must not exceed min_free_gb']
        return []


@dataclass(frozen=True)
class RecorderConfig:
    """recorder_node."""

    recordings_dir: str = param(
        '~/x30/recordings', live=False, doc='a session per <YYYYMMDD>/<HHMMSS>_<reason>/'
    )
    pose_topic: str = param(
        '/localization/pose', live=False, doc="the robot's pose of each frame (map frame)"
    )
    site_topic: str = param('/site/status', live=False, doc='the active site, in session.json')
    queue_size: int = param(
        64, min=4, max=10000, live=False, doc='files waiting for the disk; more are dropped'
    )
    thermal: ThermalConfig = nested(ThermalConfig)
    video: VideoConfig = nested(VideoConfig)
    acoustic: AcousticConfig = nested(AcousticConfig)
    patrol: PatrolConfig = nested(PatrolConfig)
    disk: DiskConfig = nested(DiskConfig)
