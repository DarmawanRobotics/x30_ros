# x30_recorder

The inspection cameras recorded on the X30's perception host (.105), where their drivers run:
the raw thermal frames of the HIKMICRO PTZ camera
([hikmicro_driver](https://github.com/DarmawanRobotics/hikmicro_driver)) for analysis, its
visible and thermal video, and the data of the Hertzino HZ-FA-371 acoustic camera
([hertzino_driver](https://github.com/DarmawanRobotics/hertzino_driver)). Everything is written
to this host's disk as it arrives: nothing waits on the network, nothing is lost when the link
to the operator drops, and the navigation host carries none of it.

| Interface | Type | |
|---|---|---|
| `recorder/start` | `std_srvs/Trigger` | start a manual session (its folder in the answer); a patrol session goes on as manual |
| `recorder/stop` | `std_srvs/Trigger` | end the session (a route still running starts a patrol session) |
| `recorder/status` | `x30_msgs/RecorderStatus`, latched, 1 Hz | recording, why, the session's folder and duration, what it holds, each stream's state, the free disk |
| in | `navigation/status` | a session runs while the path follower runs a route |
| in | `/hikmicro/thermal/temperature` (32FC1, degC), `/hikmicro/ptz/state` | the raw frames and the camera's pan, tilt and zoom |
| in | `/localization/pose`, `site/status` | the robot's pose for each frame, the active site |
| in | `/hertzino/spectrum`, `prpd`, `voiceprint`, `detection` | the acoustic camera's data |
| RTSP | the camera's channels | the video, copied as the camera encodes it (no re-encoding) |

## Sessions

A **patrol** session runs while the path follower runs a route (`navigation/status` from joining
the route to off path, paused and blocked included) and `patrol/stop_after_sec` (30 s) after it
reached its goal or failed, so a lap after a lap stays one session. A follower that went silent
for `patrol/silence_sec` ends it. A **manual** session runs between `recorder/start` and
`recorder/stop`:

```bash
rosservice call /recorder/start   # message: the session's folder
rosservice call /recorder/stop
```

Each session is a folder `<recordings_dir>/<YYYYMMDD>/<HHMMSS>_<patrol|manual>/`:

```
session.json            reason, started, stopped, site, thermal_frames, acoustic_chunks, ...
thermal/<YYYYMMDD_HHMMSS_ms>.npz        one raw frame each (below)
video/<stream>_<YYYYMMDD_HHMMSS>.mp4    segments of segment_sec (300 s) per stream
video/<stream>.log                      the stream's ffmpeg output (the password masked)
acoustic/spectrum_<stamp>.npz           chunk_sec (10 s) of spectra
acoustic/prpd_<stamp>.npz, voiceprint_<stamp>.npz
acoustic/detections.jsonl               one detection per line
```

`session.json` is rewritten as the session grows, so a session cut by a power loss still says
what it holds. The MP4 segments are fragmented: a segment cut by a power loss plays up to its
last fragment.

## Raw thermal frames

One `.npz` per thermometry frame (`radiometric_rate` of hikmicro_driver, 1 Hz), written to a
temporary file and renamed, so a file is whole or absent:

| Array | | |
|---|---|---|
| `temperature` | float32 (H, W) | degC per pixel, as the camera measured it |
| `stamp` | float64 | the frame's time (s, Unix) |
| `pose` | float64 (3) | x, y (m), yaw (rad) in the map frame; NaN without a pose within 2 s of the frame |
| `ptz` | float64 (3) | the camera's pan, tilt (deg) and zoom; NaN when its state is unknown |
| `meta` | JSON string | frame_id, ptz_moving, site |

```python
from x30_utils.io import read_thermal

frame = read_thermal('thermal/20261121_101500_250.npz')
frame['temperature'].max(), frame['pose'], frame['ptz'], frame['meta']['site']
```

or with numpy alone: `np.load(path)['temperature']`.

The inspection reports of x30_inspection keep a raw frame per target too
(`<target>_thermal.npz` next to its images).

## Video

One ffmpeg per stream copies the camera's RTSP stream into MP4 segments (`-c copy`: no
decoding, little CPU). `video/streams` are `name:channel` of the camera (`visible:101` and
`thermal:201`, its main streams); `video/urls` adds any other input as `name=url`. The
camera's password comes from the `HIKMICRO_PASSWORD` environment variable, never a parameter
(as hikmicro_driver's); it is masked in every log. A stream that stops (camera restarting, link
lost) is started again after `restart_sec`, doubling up to 60 s while it keeps failing, and is
shown as `restarting` in `recorder/status`.

## Disk

Before a session starts and every `disk/check_sec` (30 s), the oldest sessions are removed
until `disk/min_free_gb` (10 GB) is free, and those older than `disk/keep_days` (0: kept) too;
the running session is never removed. Below `disk/stop_free_gb` (3 GB) nothing is recorded and
`recorder/status` says why. The writer of the frames and chunks runs in its own thread with a
bounded queue (`queue_size`): a disk too slow drops a frame, counted as `dropped`, and never
stalls the subscriptions.

## Launch

On the perception host (`x30_bringup perception_host.launch`, `recorder:=true` by default):

```bash
export HIKMICRO_PASSWORD=...   # for the video
roslaunch x30_recorder recorder.launch                       # recordings in ~/x30/recordings
roslaunch x30_recorder recorder.launch recordings_dir:=/data/recordings host:=192.168.1.64
```

`params_file` is read after `config/recorder.yaml`: only what it changes.

## Tests

`python3 -m pytest -q -p no:cacheprovider test` (the ROS tests start a master; the video tests
need ffmpeg).
