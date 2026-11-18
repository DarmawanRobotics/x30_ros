# x30_health

One health signal for the X30, the ROS Noetic port of
[dharma_health](https://github.com/DarmawanRobotics/dharma_health): `health_monitor_node`
watches the sensor topics, the expected nodes, the stack's typed states, the safety scan, the
TF tree, the clock and the computer, and publishes `health/status` (latched) with a speed scale
for the control mux, plus `diagnostics`.

| Interface | Type | |
|---|---|---|
| `health/status` | `x30_msgs/HealthStatus`, latched | `OK` / `WARN` / `ERROR`, the issues, `speed_scale` (any error: 0) |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | one status per check (`health: <item>`) |
| in | `sensors/topics`, `odometry/status`, `localization/status`, `control/status`, `perception/safety_scan`, TF | |

The control mux scales every command by `speed_scale` with its `use_health_limit: true`
(x30_control).

## Checks

| Item (diagnostic name) | WARN | ERROR |
|---|---|---|
| `sensor:<topic>` | not published; below `rate_tolerance` × `min_hz`; header stamps older than `max_age_sec` or ahead of the clock | |
| `state:odometry` | `INITIALIZING`, `DEGRADED`; no or stale status | `LOST` |
| `state:localization` | `NONE` (not localized), `UNCERTAIN`; no or stale status | |
| `state:control` | no or stale status | e-stop engaged; field `STALE` or `BLIND` |
| `state:safety_scan` | some sectors blocked; none received during the start-up grace | stale, blind (every sector blocked) |
| `node:<fully qualified name>` | not running yet (start-up grace) | not running |
| `tf:<parent>-><child>` | late (cause: `clock`, `load`, `stopped`), ahead of the clock, gone; a required edge missing | |
| `tf:chain <odom>-><base>` | no transform | |
| `system:temperature`, `memory`, `disk`, `load`, `cpu`, `gpu` | above (disk: below) the limit; **unreadable** (never silently OK) | |
| `clock:ntp` (optional) | `chronyc tracking` offset above `max_offset_sec`, not synchronised, command failed | |

- **Stamps, not only rates.** The sensor topics are subscribed raw, any type (no message package
  needed: the Livox `CustomMsg` too): the header stamp is read from the ROS 1 bytes, and whether
  the type starts with a header from the definition in the connection header. A 10 Hz topic with
  2 s old stamps is late; negative ages mean the source clock is ahead.
- **TF diagnosis.** Late but steady ages with fresh updates are a clock offset of the source
  (`camera_init->body 1.8 s old, steady (source clock offset?)`); ages that vary or grow are late
  delivery; a stamp that stops advancing is a stopped publisher.
- **Nodes** come from the master (`getSystemState`), fully qualified: the same name in another
  namespace does not count.
- A limit of 0 turns a system reading off; an empty state topic turns that check off.

## Parameters

Defaults in [core/config.py](src/x30_health/core/config.py), the X30's in
[config/health.yaml](config/health.yaml): `/livox/lidar` at 5 Hz, `/imu/data` at 100 Hz,
x30_driver's `/x30/odom` at 20 Hz, half speed while the LiDAR, FAST-LIO or a TF edge is late.
`x30_bringup` sets `nodes/expected` and the state topics for the layers it starts.

## Launch

```bash
roslaunch x30_health health.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rostopic echo /health/status
```

Arguments: `params_file`, `robot_params`, `respawn`, `output`.

## Tests

`catkin_make run_tests_x30_health`: every check (unit), and the node on a private rosmaster:
old stamps, not localized, then OK, then odometry lost; a blind safety scan; an expected node in
another namespace; a TF edge with a steady clock offset; live parameters; the stamp of a real
serialized message.
