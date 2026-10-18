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
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | `connected`, `basic_state_name`, `gait_name`, `control_mode`, `emergency_source`, `errors`, `body_height`, `battery`, temperatures, CPU, mileage and times, `position_control`, `elevation_mode`, `brake_mode`, `velocity_source`, `charge_state`, `cmd`, `cmd_vel_ignored`, struct layouts; level ERROR without reports or on IMU timeout / driver failure, WARN on the other errors, emergency stop, low battery or ignored `cmd_vel` |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | the same status as `x30_driver`, plus `rx`, `tx`, errors and unknown report codes |
| pub | `/cmd_vel_corrected` | `geometry_msgs/Twist` | `velocity_mode: topic` only (`velocity_topic`) |

## Services (`std_srvs/Trigger`)

Codes with the default prefix `0x31`. Each service checks the robot's state first and waits
until the robot reports the change (`stand_timeout_sec`, `transition_timeout_sec`,
`feedback_timeout_sec`).

| Service | Sends | Notes |
|---|---|---|
| `stand_up` | stand/sit `0x31010202`, then torque control `0x3101020A` | sitting → initial standing → torque-controlled standing (`auto_torque_control`); nothing to do when standing |
| `sit_down` | stop stepping `0x31010201`, stand/sit `0x31010202` | from stepping it zeroes the velocity and stops stepping first; refused while walking |
| `torque_control` | `0x3101020A` | initial standing → torque-controlled standing |
| `start_stepping`, `stop_stepping` | toggle `0x31010201` | sent only from the state it leaves (a toggle) |
| `non_manual_mode`, `manual_mode` | `0x31010C03`, `0x31010C02` | waits for `is_nav_mode` in `0x1008` |
| `height_normal`, `height_crawl` | `0x31010406` value 2 / 0 | no crawl in off-road, stair and run; no change in L-Walk, Mountain, Silent; waits for `0x11050F08` |
| `gait_walk`, `gait_slope`, `gait_off_road` | `0x31010300`, `0x31010402`, `0x31010401` | stepping state only; waits for `gait_state` |
| `gait_stair`, `gait_stair_multi_frame`, `gait_stair_45` | `0x31010405`, `0x3101040A`, `0x3101040B` | and the matching elevation map mode (solid / grid / no risers; multi-frame), when the perception host reports one |
| `gait_l_walk`, `gait_mountain`, `gait_silent` | `0x31010420`, `0x31010421`, `0x31010422` | |
| `emergency_stop` | `0x31010C0E` | soft emergency stop; zeroes the velocity first |
| `save_data` | `save_data_code` (`0x010C01`, or `18` before jy_exe v2.2.45), no prefix | the robot saves 5 minutes of data and **exits its motion program** |
| `elevation_solid_ground`, `elevation_grid_ground`, `elevation_stairs_no_risers` | `0x3101EE01` value 3 / 4 / 5 to `:43899` | waits for `0x3100EE01` |
| `obstacle_decelerate`, `obstacle_detour` | `0x3101EE02` value 1 / 2 | waits for `0x3100EE02` |
| `velocity_source_joystick`, `velocity_source_navigation` | `0x3101EE03` value 1 / 2 | waits for `0x3100EE03` |
| `obstacle_height_8cm`, `obstacle_height_28cm` | `0x3100EE04` value 1 / 2 | the document's code; no feedback |
| `charge_start`, `charge_stop`, `charge_clear` | `0x91910250` value 1 / 0 / 2 to `:3333` | answers with `charge_manager_state`; `0x91910253` asks for it every `charge_query_sec` |

The multi-frame elevation map mode needs the LiDAR-inertial odometry started first (`0xBAA0001`
to port 60000, 1.5.2.4) and the robot standing still: the driver leaves that procedure to the
operator (Appendix C).

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
they are scaled like walk (`state` shows `gait_speeds`). Values with |value| < 655 count as zero
(the dead zone): a non-zero command below it goes out as ±656. REP-103 has y left and z
counter-clockwise, the robot + right and + turn right: lateral and turn are negated
(`invert_vy` / `invert_wz` flip them again if the real robot disagrees).

The axes only move the robot in the **stepping** state; in torque-controlled standing the same
commands change the body's pose (1.2.3.1), so the driver sends nothing then and shows why in
`cmd_vel_ignored`. The robot obeys the autonomous side only in **non-manual mode** (1.2.4).
When `cmd_vel` asks for motion:

- `auto_non_manual_mode`: `0x31010C03` while `0x1008` does not report non-manual mode, at most
  every `mode_refresh_sec`;
- `auto_stepping`: the stepping toggle `0x31010201` from torque-controlled standing, at most
  every `stepping_retry_sec` (never while stepping: it would stop);
- `idle_stop_stepping_sec` (0: off): stop stepping again after that long without motion.

When `cmd_vel` goes quiet for `cmd_timeout_sec` the driver sends zeros for `stop_burst_sec`, then
nothing (the robot decelerates to zero after 1 s without an axis command).

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
| `heartbeat_hz`, `connect_retry_sec` | 5.0, 5.0 | heartbeat; connect again while no reports come |
| `command_prefix` | `0x31` | `0x31` autonomous module, `0x21` remote control |
| `save_data_code` | `0x010C01` | `18` for jy_exe older than v2.2.45 |
| `perception_host_ip` | `192.168.1.105` | empty: no perception services |
| `elevation_map_port`, `charge_port` | 43899, 3333 | |
| `elevation_query_sec`, `charge_query_sec`, `feedback_timeout_sec` | 5.0, 2.0, 2.0 | |
| `velocity_mode`, `velocity_topic` | `axis`, `/cmd_vel_corrected` | |
| `cmd_rate_hz`, `cmd_timeout_sec`, `stop_burst_sec` | 50.0, 0.5, 0.5 | velocity stream and watchdog |
| `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.0 | clamp of `cmd_vel` |
| `axis_dead_zone` | 655 | |
| `invert_vy`, `invert_wz` | false | sign fixes, see above |
| `auto_non_manual_mode`, `mode_refresh_sec` | true, 2.0 | |
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
(elevation map modes with their 10 Hz feedback, charging states):

```bash
rosrun x30_driver fake_robot.py --start sitting      # or standing, stepping; --layout pack1
roslaunch x30_driver driver.launch overrides_file:=$(rospack find x30_driver)/config/fake_robot.yaml
```

## Test

```bash
catkin_make run_tests_x30_driver && catkin_test_results build/test_results
python3 -m pytest test        # without ROS: codec, policy, link, simulated robot, parameters
```

The codec is checked against the document's worked example (Appendix A), every command code the
document prints, the struct sizes and field offsets of each layout and Appendix B; the velocity
policy, the UDP link, the simulated robot and the parameter file have their own tests;
`test/smoke.test` (rostest) runs the node against the simulated robot.
