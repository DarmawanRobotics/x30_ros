# x30_driver

ROS Noetic driver of the Deep Robotics Jueying X30. It speaks the motion host's UDP protocol
(*Jueying X30 API Documentation* V1.0.5-0, 1.1 to 1.3): `cmd_vel` as axis commands in the
stepping state, the reports as ROS topics, every robot command as a `std_srvs/Trigger` service;
and the perception host's elevation map and autonomous charging commands (1.4, 1.5).

## Nodes

| Node | Executable | |
|---|---|---|
| `x30_driver` | `driver_node` | UDP links to the motion and perception hosts, ROS topics and services |
| `fake_x30` (tests) | `fake_robot.py` | simulated motion and perception hosts on UDP, no ROS |

## Usage

```bash
roslaunch x30_driver driver.launch                       # namespace x30
rostopic echo /x30/state
rosservice call /x30/stand_up
rostopic pub -r 20 /x30/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'
```

## Topics (in the launch namespace, `x30` by default)

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | `base_link`, REP-103: `linear.x` forward, `linear.y` left, `angular.z` counter-clockwise; zeros after `cmd_timeout_sec` without a command, then silence |
| pub | `odom` | `nav_msgs/Odometry` | from `0x1009`: pose from `leg_odom_pos` {x, y, yaw} (world = the pose at start-up), roll and pitch from the IMU; twist in `base_link` from `leg_odom_vel`; **no TF** unless `publish_tf` |
| pub | `imu` | `sensor_msgs/Imu` | from `0x100A`: roll, pitch, yaw (degrees → quaternion), angular velocity, acceleration; frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | from `0x100A`: `FL_HipX_joint`, `FL_HipY_joint`, `FL_Knee_joint`, `FR_...`, `HL_...`, `HR_...` (the official model's names, the report's order): position, velocity, effort = torque |
| pub | `battery` | `sensor_msgs/BatteryState` | from `0x21050F0A`: `percentage` 0–1, voltage, current, charge and capacity (10 mAh units → Ah), the hottest of the four temperatures |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | `connected`, `basic_state_name`, `gait_name`, `control_mode`, `emergency_source`, `errors`, `body_height`, `battery`, temperatures, CPU, mileage and times, `position_control`, `elevation_mode`, `brake_mode`, `velocity_source`, `charge_state`, `cmd`, `cmd_vel_ignored`, `armed`, `disarmed_by`, `obeyed_in`, `gait_speeds`, struct layouts; level ERROR without reports or on IMU timeout / driver failure, WARN on the other errors, emergency stop, low battery or ignored `cmd_vel` |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | the same status as `x30_driver`, plus `rx`, `tx`, errors and unknown report codes |
| pub | `/cmd_vel_corrected` | `geometry_msgs/Twist` | `velocity_mode: topic` only (`velocity_topic`) |

## Services (`std_srvs/Trigger`)

Codes with the default prefix `0x31`. Each service checks the robot's state first and succeeds
only when the robot reports the change (`stand_timeout_sec`, `transition_timeout_sec`,
`feedback_timeout_sec`). The posture services run one at a time (a second call meanwhile is
refused as busy), and stand/sit and stepping, being toggles, go out only once the reported
state shows the previous one (or after `transition_timeout_sec`).

| Service | Sends | Notes |
|---|---|---|
| `stand_up` | stand/sit `0x31010202`, then torque control `0x3101020A` | sitting → initial standing → torque-controlled standing (`auto_torque_control`); nothing to do when standing |
| `sit_down` | stop stepping `0x31010201`, stand/sit `0x31010202` | from stepping it stops stepping first; refused while walking; holds the velocity stream (see Velocity) |
| `torque_control` | `0x3101020A` | initial standing → torque-controlled standing |
| `start_stepping` | toggle `0x31010201` | only from torque-controlled (or initial) standing |
| `stop_stepping` | toggle `0x31010201` | only while stepping; holds the velocity stream, so `cmd_vel` does not start stepping again |
| `non_manual_mode`, `manual_mode` | `0x31010C03`, `0x31010C02` | waits for `is_nav_mode` in `0x1008`; leaving the mode the driver is obeyed in holds the velocity stream, asking for it releases it |
| `height_normal`, `height_crawl` | `0x31010406` value 2 / 0 | no crawl in off-road, stair and run, nor without a fresh motion state report; no change in L-Walk, Mountain, Silent; fails when `0x11050F08` still shows the old height (succeeds without any height report) |
| `gait_walk`, `gait_slope`, `gait_off_road` | `0x31010300`, `0x31010402`, `0x31010401` | stepping state only; waits for `gait_state` |
| `gait_stair`, `gait_stair_multi_frame`, `gait_stair_45` | `0x31010405`, `0x3101040A`, `0x3101040B` | and the matching elevation map mode (solid / grid / no risers; multi-frame), when the perception host reports one |
| `gait_l_walk`, `gait_mountain`, `gait_silent` | `0x31010420`, `0x31010421`, `0x31010422` | Silent has no speeds in Appendix B: `cmd_vel` moves the robot in it only with `gait_limits` |
| `emergency_stop` | `0x31010C0E`, three times | soft emergency stop; holds the velocity stream; succeeds once the robot reports emergency stop or fall |
| `save_data` | `0x31010C01` (prefix + `save_data_code` `0x010C01`), or `18` as it is before jy_exe v2.2.45 | only while sitting or after an emergency stop: the robot saves 5 minutes of data and **exits its motion program** |
| `elevation_solid_ground`, `elevation_grid_ground`, `elevation_stairs_no_risers` | `0x3101EE01` value 3 / 4 / 5 to `:43899` | waits for `0x3100EE01`; from multi-frame mode (18, 20) then `0x0BAA0001` value 0 to `:60000` to turn the LiDAR-inertial odometry off (1.5.2.5) |
| `obstacle_decelerate`, `obstacle_detour` | `0x3101EE02` value 1 / 2 | waits for `0x3100EE02` |
| `velocity_source_joystick`, `velocity_source_navigation` | non-manual mode, then `0x3101EE03` value 1 / 2 | 1.5.1.1: non-manual mode first; waits for `0x3100EE03` |
| `obstacle_height_8cm`, `obstacle_height_28cm` | `obstacle_height_code` value 1 / 2 | `0x3100EE04` as the document prints it; no feedback over UDP: check `/step_z_max_state` on the perception host, and try `0x3101EE04` if it does not change |
| `charge_start`, `charge_stop`, `charge_clear` | `0x91910250` value 1 / 0 / 2 to `:3333` | answers with `charge_manager_state`; `0x91910253` asks for it every `charge_query_sec` |

Multi-frame elevation map mode is entered by hand (1.5.2.4, Appendix C): with the robot standing
still, `0xBAA0001` value 1 to port 60000 (answer 0), then mode 20 to port 43899; the map reports
18 (preparing) and 20 once the odometry runs. The elevation services leave it again: the new
mode first, then the odometry off once the map reports that mode (1.5.2.5).

## Velocity

`cmd_vel` is clamped to `max_vx`, `max_vy`, `max_wz` and sent at `cmd_rate_hz` (50 Hz, 1.2.3) as
three axis commands: forward `0x31010130`, lateral `0x31010131`, turn `0x31010135`. The robot
reads them as fractions of the current gait's maximum speeds (1.2.3.2, Appendix B):

| Gait | forward | backward | lateral | yaw |
|---|---|---|---|---|
| walk | 1.2 m/s | 0.75 m/s | 0.15 m/s | 0.45 rad/s |
| slope | 0.7 | 0.75 | 0.25 | 0.5 |
| off-road | 0.3 | 0.4 | 0.1 | 0.5 |
| stair (solid, grid, no risers) | 0.3 | 0.3 | 0.2 | 0.8 |
| stair (multi-frame) | 0.6 | 0.3 | 0.2 | 0.8 |
| 45° stair | 0.3 | 0.2 | 0.2 | 0.8 |
| L-Walk | 1.0 | 1.0 | 0.5 | 1.2 |
| Mountain | 1.0 | 1.0 | 0.8 | 1.2 |

so the driver sends `value = v / vmax × 32767` for the gait in `0x1009`, clamped to ±32767. At
crawl height walk and slope run at half their forward speed. Run and Silent are not in the table:
the driver sends zeros in them (`cmd_vel_ignored` says why) unless `gait_limits` gives their
speeds, e.g. `{silent: [0.5, 0.5, 0.2, 0.5]}` (forward, backward, lateral m/s, yaw rad/s; it can
also replace Appendix B gaits; `state` shows `gait_speeds`). Values with |value| < 655 count as
zero (the dead zone): a velocity of at least 0.001 m/s (rad/s) that falls below it goes out as
±656, anything smaller as 0. REP-103 has y left and z counter-clockwise, the robot + right and +
turn right: lateral and turn are negated (`invert_vy` / `invert_wz` flip them again if the real
robot disagrees).

The axes only move the robot in the **stepping** state; in torque-controlled standing the same
commands change the body's pose (1.2.3.1), so the driver sends nothing then, nor without a fresh
`0x1009` (younger than `state_timeout_sec`), and shows why in `cmd_vel_ignored`. The robot obeys
each source in one control mode (1.2.4): `0x31` (autonomous module, default) in **non-manual
mode**, `0x21` (remote control: the driver acts as the handheld) in **manual mode**; `state`
shows it as `obeyed_in`. When `cmd_vel` asks for motion:

- `auto_control_mode`: that mode (`0x31010C03`, or `0x21010C02` with `0x21`) while `0x1008`
  does not report it, at most every `mode_refresh_sec`;
- `auto_stepping`: the stepping toggle `0x31010201` from torque-controlled standing, at most
  every `stepping_retry_sec` (never while stepping: it would stop);
- `idle_stop_stepping_sec` (0: off): stop stepping again after that long without motion.

The driver **holds back** (no switches, only zeros while the robot steps; `armed: False`,
`disarmed_by` in `state`):

- after an operator takeover: the robot was in the mode that obeys the driver and left it while
  `cmd_vel` moved it (e.g. manual on the handheld);
- after `stop_stepping`, `sit_down`, `emergency_stop`, `manual_mode` (with `0x31`; with `0x21`:
  `non_manual_mode` and the velocity source services);
- while the robot reports emergency stop or fall.

It goes on once `cmd_vel` has been quiet or zero for `stop_burst_sec` (or a mode service asks for
the mode that obeys it). When `cmd_vel` goes quiet for `cmd_timeout_sec` the driver sends zeros
for `stop_burst_sec`, then nothing (the robot decelerates to zero after 1 s without an axis
command). On shutdown it stops the stream and sends zero axes three times.

`velocity_mode: topic` publishes `cmd_vel` to `velocity_topic` (`/cmd_vel_corrected`) instead:
in non-manual mode the motion program executes that topic (2.3). It is for a driver started
against the X30's own ROS master; the switches above still go out over UDP.

## Reports and struct layouts

The document gives plain C structs. The codec recognises each one by its length:

| Report | Code | natural | pack(1) |
|---|---|---|---|
| `RcsData` (name, mileage, times, handheld axes, mode, emergency source, errors) | `0x1008` | 88 bytes (72 with a 4-byte `long`) | 85 (69) |
| `MotionStateData` (basic state, gait, leg odometry) | `0x1009` | 60 | 56 |
| `ControllerSensorData` (IMU, joint positions, velocities, torques) | `0x100A` | 184 | 184 |
| `ControllerSafeData` (motor and driver temperatures, CPU) | `0x100B` | 68 | 68 |
| `BatterySensorData` | `0x21050F0A` | 40 | 39 |
| body height (simple command: -1 crawl, 0 normal) | `0x11050F08` | | |

Any other length is logged once and skipped; `state` shows the layouts in use. The battery
voltage is "V" in the document (a `uint16`): `battery_voltage_scale: 0` reads it as volts, or as
mV / 10 mV when the value is too large to be volts. The current is in A before Deeprcs 1.10.19,
in 10 mA from 1.10.19 (`battery_current_scale`).

## Parameters

[config/driver_params.yaml](config/driver_params.yaml), every one commented (a test checks the
file against the node):

| Parameter | Default | |
|---|---|---|
| `motion_host_ip`, `motion_host_port` | `192.168.1.103`, `43893` | where commands go |
| `local_ip`, `local_port` | `0.0.0.0`, `43897` | where reports arrive (listed in the motion host's `network.toml`) |
| `heartbeat_hz`, `connect_retry_sec` | 5.0, 5.0 | heartbeat (at least 2 Hz); connect again while no reports come |
| `command_prefix` | `0x31` | `0x31` autonomous module (obeyed in non-manual mode), `0x21` remote control (manual mode); anything else stops the node |
| `save_data_code` | `0x010C01` | sent with the prefix; `18` (sent as it is) for jy_exe older than v2.2.45 |
| `perception_host_ip` | `192.168.1.105` | empty: no perception services |
| `elevation_map_port`, `charge_port`, `lio_port` | 43899, 3333, 60000 | |
| `obstacle_height_code` | `0x3100EE04` | as the document prints it; `0x3101EE04` is the likely intended code |
| `elevation_query_sec`, `charge_query_sec`, `feedback_timeout_sec` | 5.0, 2.0, 2.0 | |
| `velocity_mode`, `velocity_topic` | `axis`, `/cmd_vel_corrected` | |
| `cmd_rate_hz`, `cmd_timeout_sec`, `stop_burst_sec` | 50.0, 0.5, 0.5 | velocity stream and watchdog |
| `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.0 | clamp of `cmd_vel` |
| `axis_dead_zone` | 655 | |
| `gait_limits` | `{}` | speeds of gaits Appendix B leaves out (run, silent) or overrides |
| `invert_vy`, `invert_wz` | false | sign fixes, see above |
| `auto_control_mode`, `mode_refresh_sec` | true, 2.0 | switch to the mode that obeys the driver |
| `auto_stepping`, `stepping_retry_sec`, `idle_stop_stepping_sec` | true, 2.0, 0.0 | |
| `auto_torque_control`, `stand_timeout_sec`, `transition_timeout_sec` | true, 10.0, 5.0 | posture services |
| `odom_frame`, `base_frame`, `imu_frame`, `publish_tf` | `odom`, `base_link`, `imu_link`, false | |
| `odom_hz`, `imu_hz`, `joint_state_hz`, `state_hz`, `battery_hz` | 50, 100, 50, 10, 1 | |
| `state_timeout_sec`, `low_battery` | 1.0, 0.2 | |
| `battery_voltage_scale`, `battery_current_scale` | 0.0, 0.01 | see above |

## Launch

`launch/driver.launch`:

| Argument | Default | |
|---|---|---|
| `namespace` | `x30` | topics and services |
| `params_file` | [config/driver_params.yaml](config/driver_params.yaml) | a copy with this robot's settings |
| `overrides_file` | empty | a few parameters on top, e.g. [config/fake_robot.yaml](config/fake_robot.yaml) |
| `respawn`, `output` | `true`, `screen` | |

## Simulated robot

`test/fake_robot.py` plays the motion host (posture state machine, stepping, modes, gaits, body
height, axis commands integrated into the leg odometry, every report) and the perception host
(elevation map modes with their 10 Hz feedback to every client, the LiDAR-inertial odometry
switch, charging states). It obeys `0x31` axis commands in non-manual mode and `0x21` ones in
manual mode only:

```bash
rosrun x30_driver fake_robot.py --start sitting      # or standing, stepping; --layout pack1
roslaunch x30_driver driver.launch overrides_file:=$(rospack find x30_driver)/config/fake_robot.yaml
```

## Test

```bash
catkin_make run_tests_x30_driver && catkin_test_results build/test_results
python3 -m pytest test        # without ROS: codec, policy, stream, link, simulated robot, node
```

The codec is checked against the document's worked example (Appendix A), every command code the
document prints, the struct sizes and field offsets of each layout and Appendix B. The velocity
policy, the automatic switches and the toggle guard have unit tests; `test_stream.py` runs the
velocity stream in a closed loop with the simulated robot in simulated time (takeover, holds
after stop services and emergency stop, toggles with late reports, unknown gaits, control mode
per prefix); `test_node.py` runs the node against the simulated robot over UDP with the rospy
stand-ins of `test/rosstub` (every service behaviour, shutdown, start-up checks);
`test/smoke.test` (rostest) runs it through the real ROS interface. The parameter file is checked
against the node.
