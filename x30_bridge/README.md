# x30_bridge

The handheld's way into the X30 without ROS: `teleop_udp_node` on the navigation host, the UDP
gateway of the [X30Controller](https://github.com/DarmawanRobotics/X30Controller) app (a Skydroid
H12 or any Android on the robot's Wi-Fi). Its wire format is the teleop protocol v1 of Pashupati
with the X30's own commands and state ([PROTOCOL.md](PROTOCOL.md)), so Pashupati's handheld
drives the X30 too. The gateway never drives the robot itself: the stick is the `handheld` source
of the control mux, which alone publishes `/x30/cmd_vel` after its envelope, protective field
and e-stop.

| Handheld | The stack |
|---|---|
| stick (`joy`) | `control/remote/handheld/cmd_vel`, clamped to `max_vx/vy/wz` and `robot/limits`; NaN or text is 0; a silent handheld is zeroed after 2 s |
| `set_mode` | `control/set_mode` (`auto`, or `remote` with source `handheld`) |
| `estop` | `control/estop_request` at once, then `control/estop/engage`; `estop_release`: `control/estop/release` |
| `nav_*` | `navigation/start`, `stop`, `pause`, `resume` |
| `record_*`, `mark_stop` | `route/start`, `route/stop`, `route/mark_stop_point`: teach a route with its stop points |
| `list_routes`, `load_route` | the routes of every site under `sites_dir`, `navigation/load_path` |
| `localize`, `record_tags` | `localization/start`, `localization/record_tags` |
| `dock`, `undock`, `trip_reset` | `docking/dock`, `docking/undock`, `odometry/trip_reset` |
| `nav_speed` | the path follower's live `speed/cruise`, between `patrol_speed_min` and `patrol_speed_max` (never above `robot/limits/max_vx`) |
| `recorder_start`, `recorder_stop` | x30_recorder's `recorder/start`, `recorder/stop` on the perception host |
| `inspect` | x30_inspection's `inspection/inspect` where the robot stands (answered when done) |
| robot commands | the driver's `std_srvs/Trigger` services `/x30/<name>` (`robot_commands`: stand up, sit down, stepping, gaits, `emergency_stop`, which also latches the software e-stop) |

The state comes from the typed status: `control/status`, `navigation/status`,
`localization/status`, `health/status` with the computer's figures of `/diagnostics`,
`navigation/waypoints`, `odometry/trip`, `recorder/status`, `inspection/status`, and the driver's
`/x30/battery`, `/x30/cmd_vel` and `/x30/diagnostics`. Service calls run on worker threads, so a
long one (an inspection takes minutes) blocks neither the stick nor the state.

Camera frames: the JPEGs of `cameras` (the HIKMICRO camera's `visible` and `thermal` streams as
`sensor_msgs/CompressedImage`, re-encoded to `camera_max_width` when bigger than
`camera_max_bytes`), the one the handheld picks, at most `camera_fps`. The cloud: FAST-LIO's
`/cloud_registered_body` in `base_link`, voxel-downsampled. Both and TF are subscribed only while
a handheld is connected and wants them.

## Security

The shared secret is `X30_TELEOP_TOKEN` in the environment of the navigation host's service
(`/etc/x30/navigation.env`, like the camera passwords on the perception host), never a parameter:
the parameter server is readable by every node and roslaunch prints it. Without it anyone on the
robot's Wi-Fi can drive the robot: the gateway warns at start. Set the same secret in the app.

## Launch

`navigation_host.launch` starts it (`bridge:=false` leaves it out), `bringup.launch bridge:=true`
elsewhere (the simulator too):

```bash
export X30_TELEOP_TOKEN=...   # the app's token
roslaunch x30_bridge bridge.launch                  # UDP 9870, the driver in /x30
roslaunch x30_bridge bridge.launch port:=9871 robot_ns:=x30 sites_dir:=~/x30/map
```

Arguments: `params_file` (on top of [config/bridge.yaml](config/bridge.yaml)), `robot_params`,
`sites_dir`, `robot_ns`, `port`, `respawn`, `output`.

## Layout

| Module | |
|---|---|
| `teleop_udp_node.py` | messages, commands, the state message |
| `core/protocol.py` | wire format: JSON, chunked streams, packed points |
| `core/session.py` | one driver at a time, acknowledgements, the `denied` rate limit |
| `core/commands.py` | which service each command calls, its time limit, checks of the arguments |
| `core/status.py` | the stack's status as protocol v1 names it, the recorder and substation parts |
| `core/routes.py` | taught routes inside the sites directory |
| `core/config.py` | `TeleopConfig` |
| `utils/link.py` | the UDP socket with its reader thread |
| `utils/clients.py` | the service calls on worker threads, the patrol speed |
| `utils/telemetry.py` | the status subscriptions |
| `utils/streams.py` | camera, cloud and pose, subscribed only while wanted |

## Tests

`python3 -m pytest -q -p no:cacheprovider test` (the ROS tests start a master and drive the node
with real UDP sockets against fakes of the stack's services).
