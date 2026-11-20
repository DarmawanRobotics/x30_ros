# Teleop UDP protocol (v1) on the X30

The X30 speaks the teleop protocol v1 of Pashupati
([its PROTOCOL.md](https://github.com/DarmawanRobotics/Pashupati/blob/main/src/pashupati_bridge/PROTOCOL.md)):
the same datagrams, so a handheld of either robot drives the other. What the X30 adds is marked
**X30**; a v1 handheld ignores it. Pashupati's own parts (its VLM inspection, people detection,
AprilTag overlay, internet check, WebRTC video) are absent here: their fields are `null` or empty.

One UDP port on the robot's navigation host (default `9870`, 192.168.1.106 on the robot's Wi-Fi).
The handheld sends JSON datagrams; the robot answers with JSON datagrams (control) and chunked
binary datagrams (streams).

## Session

1. The handheld sends `hello` (and again every 1 s as a keep-alive when idle).
2. The robot answers `welcome` and the loaded route, then streams `state` at `telemetry_hz`.
3. Any datagram refreshes the session; after `client_timeout_sec` (2 s) of silence the robot drops
   the handheld and zeroes the stick.
4. One driver at a time: a `hello` from another device (IP address) is answered with `busy` while
   the current one is alive. The same device may come back from a new port (app restart).

With a shared secret (`X30_TELEOP_TOKEN` in the robot's environment, never a parameter) every
message must carry `"k": "<token>"`, compared in constant time; a `hello` without it is answered
with `denied` (once a second per address), anything else is dropped. Malformed datagrams are
dropped; stick values that are not finite numbers count as 0.

## Discovery

Broadcast `{"t": "discover"}` to UDP 9870 (no token needed): every robot answers
`{"t": "beacon", "name", "robot": "x30", "secured", "in_use_by", "mode", "battery"}`.

## Handheld → robot

| `t` | Fields | |
|---|---|---|
| `hello` | `name`, `streams: {camera, lidar, video: bool, camera_fps?, camera_name?}` | register / keep alive |
| `joy` | `vx`, `vy` (m/s), `wz` (rad/s) | the stick at 20–30 Hz; drives only in `remote` mode |
| `cmd` | `id` (int or text, unique per command), `name`, `args` | a command, below |
| `streams` | `camera`, `lidar`, `video` (bool), `camera_fps`, **X30** `camera_name` | change the streams |
| `ping` | `ts` | echoed as `pong` |
| `bye` | | leave |

A command runs once per `id` and `name` in a session: resend the same `cmd` until its `ack`
comes (every 200 ms). A long one (`inspect`, `nav_start`) is answered `ok: null` ("pending")
while it runs, then with its outcome.

| `name` | `args` | Service of the stack |
|---|---|---|
| `estop`, `estop_release` | | `control/estop/engage` (latched at once through `control/estop_request`), `release` |
| `set_mode` | `mode`: `auto` \| `remote` | `control/set_mode`, source `handheld` |
| `nav_start`, `nav_stop`, `nav_pause`, `nav_resume` | | `navigation/start` ... |
| `record_start`, `record_stop` | | `route/start`, `route/stop`: teach a route |
| `mark_stop` | `dwell` (s, 0–3600) | `route/mark_stop_point` |
| `list_routes` | | `ack.data`: the routes of every site, newest first (`gardu/routes/loop.csv`) |
| `load_route` | `file` | `navigation/load_path`, a path from `list_routes` only |
| `get_route` | | the route stream again |
| `localize`, `record_tags` | | `localization/start`, `localization/record_tags` |
| `dock`, `undock` | | `docking/dock`, `docking/undock` |
| `nav_speed` | `speed` (m/s, `state.speed.min`–`max`) | the follower's `speed/cruise` |
| `trip_reset` | | `odometry/trip_reset` |
| **X30** `recorder_start`, `recorder_stop` | | `recorder/start`, `recorder/stop` (x30_recorder) |
| **X30** `inspect` | | `inspection/inspect`: the inspection point where the robot stands (minutes) |
| `stand_up`, `sit_down`, `start_stepping`, `stop_stepping`, `gait_walk`, `gait_off_road`, `gait_stair`, `emergency_stop` | | the driver's Trigger services (`robot_commands`); `emergency_stop` also latches the e-stop |

## Robot → handheld (JSON)

| `t` | Fields |
|---|---|
| `welcome` | `name`, **X30** `robot: "x30"`, `commands` (every name), `robot_commands` (the driver's, in order), `limits` ([vx, vy, wz] maxima), **X30** `cameras` (names of the camera streams, the first shown by default) |
| `ack` | `id`, `ok` (true, false, `null` while pending), `msg`, optional `data` |
| `state` | `seq`, `mode` (`auto` \| `remote`), `estop`, `field` (`clear`, `limited`, `stop`, `stale`, `off`, `null`), `nav` (`NAV_INACTIVE`, `JOINING`, `FOLLOWING`, `APPROACHING`, `DWELLING`, `PAUSED`, `BLOCKED`, `OFF_PATH`, `GOAL_REACHED`, `FAILED`), `nav_msg`, `mission` (`waypoint 12/240`, `paused: operator`), `localization` (`none` or the source of the last correction), `recording` (a route being taught), `pose` ([x, y, yaw] in the map frame or `null`), `battery` ({pct, v, a, temp}), `driver` ({level, msg}), `cmd` ([vx, vy, wz] to the robot), `health` ({ok, issues[], cpu, gpu, ram, ram_used, ram_total, temp}), `trip` ({m, s, v, avg, max, since}), `speed` ({target, min, max} of the patrol), **X30** `camera` (the camera streamed), **X30** `recorder`, **X30** `substation`; `tags: []`, `inspection`, `people`, `net`: `null` |
| `pong` | `ts` |
| `busy`, `denied` | `msg` |
| `bye` | the robot shuts down |

**X30** `recorder` (x30_recorder's `recorder/status`, `null` when no recorder ever ran):
`{on, reason (patrol | manual), session (its folder), sec, frames (raw thermal), chunks
(acoustic), videos (MP4 segments), dropped, streams ["visible: recording", ...], free_gb, msg,
offline (no status for 5 s)}`.

**X30** `substation` (x30_inspection's `inspection/status`, `null` without it): `{state (idle,
inspecting, disabled), site, points, targets, thermal, acoustic (cameras connected), current,
last (the last point), severity (normal, attention, warning, critical), ambient (degC), msg,
offline (no status for 10 s)}`.

## Binary streams

Every datagram starts with a 13-byte little-endian header: magic `PSH1`, u8 stream (1 camera,
2 lidar, 3 route), u32 frame number, u16 chunk index, u16 chunk count. Payloads are split into
chunks of at most 1400 bytes; reassemble per stream and drop an incomplete frame when a newer
one starts.

| Stream | Payload |
|---|---|
| 1 camera | one JPEG: the camera of `camera_name` (`visible`, `thermal` of the HIKMICRO PTZ camera), at most `camera_fps` |
| 2 lidar | N × (int16 x, y, z) in centimetres, `base_link`, FAST-LIO's scan voxel-downsampled |
| 3 route | JSON `{"t": "route", "points": [[x, y, dwell_sec], ...]}` in the map frame |

The robot subscribes to a camera, the cloud and TF only while a handheld is connected and wants
them.
