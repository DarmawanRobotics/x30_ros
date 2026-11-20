# x30_fleet

The X30 in the NETRA command center: `fleet_uplink_node`, the robot side of NETRA's robot API
v1 (the one Pashupati speaks), with the X30 in NETRA's substation module like HotDog.

- **Telemetry and events** over one WebSocket (`/api/v1/robot/ws`, the robot's token as
  `Authorization: Bearer`, its id as `X-Robot-Id`): `hello`, `telemetry` at `telemetry_hz`,
  `patrol_event` (started, lap, finished), `stop_event`, `alert` (blocked, off the path, failed,
  the e-stop), `ack`, `time_sync`. Events wait while NETRA is unreachable; the connection comes
  back with a backoff.
- **NETRA's patrols on the robot's routes**: `start_patrol` downloads the route into the active
  site (`<sites_dir>/<site>/netra/<route_id>.csv`) and its substation inspection plan
  (`netra/inspection.yaml`, inspected by x30_inspection beside the site's own points), then
  loads the route and starts the follower.
- **Uploads through a spool on disk** (`spool_dir`): x30_inspection's reports of NETRA's points
  (`POST /api/v1/robot/inspections`: the readings of every region, the thermal, visible and sound
  map images) and freshly taught routes for NETRA's route database (`POST /api/v1/robot/routes`).
  An item stays on the robot until NETRA has it, so nothing is lost offline or across restarts.
- **NETRA's commands** as the stack's services, acknowledged with the service's answer.
- **The clock**: the offset to NETRA's clock from `time_sync` exchanges, in the telemetry and
  applied to the measurements' times; a warning when the robot's clock is off by more than
  `clock_warn_ms` (chrony on the robot).

The token comes from the environment variable `token_env` (`X30_NETRA_TOKEN`), never a parameter:
`x30_host.sh install navigation` puts it in the private settings file of the navigation host.

## Run

```bash
# the navigation host (on when netra_url is set; X30_LAUNCH_ARGS in /etc/x30/navigation.env)
roslaunch x30_bringup navigation_host.launch netra_url:=https://netra.example.com netra_robot_id:=x30-01
# the uplink alone
X30_NETRA_TOKEN=... roslaunch x30_fleet fleet.launch server_url:=https://netra.example.com
# a fake NETRA on this computer (--log keeps what the robot sends; the tests run it in-process)
rosrun x30_fleet fake_netra --port 8765 --log /tmp/netra.jsonl
```

Without `server_url` the node only spools: taught routes and inspections wait on the disk.

## NETRA's commands

| command | the stack |
|---|---|
| `start_patrol {route_id, patrol_id}` | the route and its plan into the site, `navigation/load_path`, `navigation/start` |
| `stop_patrol`, `pause`, `resume` | `navigation/stop`, `navigation/pause`, `navigation/resume` |
| `localize` | `localization/start` |
| `estop`, `estop_release` | `control/estop_request` at once, then `control/estop/engage`; `control/estop/release` |
| `dock`, `undock` | `docking/dock`, `docking/undock` |
| `recorder_start`, `recorder_stop` | `recorder/start`, `recorder/stop` (x30_recorder on the perception host) |
| `inspect_point {point}` | one NETRA point where the robot stands, now (`inspection/inspect`); refused while patrolling |

## NETRA's substation plan

NETRA plans the inspection points of a route by stop: the rows of the route CSV with a dwell,
counted from 0. Each stop becomes an x30_inspection point `netra_stop<k>` at that row's position,
each NETRA point a target `netra_<id>`:

- the PTZ pose within the camera's limits (pan ±360°, tilt ±90°, zoom 1–60);
- a thermal point's targets as regions named by their key, the ROI's fractions as the box (no
  ROI: the whole frame); the lowest positive `limit_c` as the target's own limit (at most
  500 °C);
- a partial discharge point as an acoustic measurement.

A report of such a target becomes one upload of that NETRA point: `uid`
`<robot_id>.<report folder>.<point id>` (the same report never counts twice), the time on NETRA's
clock, the patrol, the pose, the ambient temperature, `thermal [{key, max_c, min_c}]` or
`pd {db, detected, kind}`, and the images as `thermal`, `visual` and `acoustic`.

## Telemetry

NETRA's fields (`mode`, `nav`, `mission`, `localization`, `battery`, `driver`, `health` with the
topic rates and the computer in NETRA's names, `estop`, `field`, `pose`, `patrol`, `clock`) and
the X30's own:

- `recorder`: the camera recorder (`on`, `reason`, `session`, `sec`, `frames`, `chunks`,
  `videos`, `dropped`, `streams`, `free_gb`, `msg`, `at`: the robot's time of its last status);
- `substation`: x30_inspection (`state`, `site`, `points`, `targets`, the cameras connected,
  `current`, `last`, `severity`, `ambient`, `msg`, `at`).

## Parameters

[`config/fleet.yaml`](config/fleet.yaml): `server_url`, `token_env`, `robot_id`, `verify_tls`,
`telemetry_hz`, `upload_period_sec`, `spool_dir`, `sites_dir`, `clock_sync_period_sec`,
`clock_warn_ms`, `battery_topic`, `health_diagnostics_topic`, `driver_diagnostics_topic`.
