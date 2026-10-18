<div align="center">

# x30_ros

**ROS Noetic robot package for the Deep Robotics Jueying X30: the motion host's UDP protocol as
a driver, the perception host's elevation map and autonomous charging as services, and Deep
Robotics' own URDF as xacro.**

[![CI](https://github.com/DarmawanRobotics/x30_ros/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/x30_ros/actions/workflows/ci.yml)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](http://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
![Python](https://img.shields.io/badge/Python-3.8-3776AB?logo=python&logoColor=white)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

[Packages](#packages) · [Network](#network) · [Quick start](#quick-start) ·
[Interface](#interface) · [Verify on the real robot](#verify-on-the-real-robot) ·
[Safety](#safety) · [Develop](#develop)

</div>

---

The X30's motion host takes commands and reports its state over UDP, and its perception host
runs the elevation map and the autonomous charging behind two more UDP ports (Deep Robotics
*Jueying X30 API Documentation* V1.0.5-0, 2025-08-21, chapter 1). This repository turns all of
it into ROS Noetic topics and `std_srvs/Trigger` services, on a computer on the robot's network
or on the robot's own computers.

Docs: [x30_driver](x30_driver/README.md) · [x30_description](x30_description/README.md) ·
[x30_bringup](x30_bringup/README.md) · [CHANGELOG](CHANGELOG.md)

## Packages

| Package | |
|---|---|
| [`x30_driver`](x30_driver/README.md) | the UDP driver: `cmd_vel` → axis commands scaled by the current gait; reports → `odom`, `imu`, `joint_states`, `battery`, `state`, `diagnostics`; every robot command, elevation map mode and charging command as a Trigger service; a simulated robot for tests and trials |
| [`x30_description`](x30_description/README.md) | URDF: Deep Robotics' X30 model with its meshes (legs, four LiDAR frames), `base_link`, `imu_link`; `description.launch`, `display.launch` |
| [`x30_bringup`](x30_bringup/README.md) | one launch for the driver and the URDF |

## Network

From the API documentation (sections 1.2 to 1.5):

| | |
|---|---|
| motion host | `192.168.1.103`, commands to UDP **43893** |
| reports | 200 Hz state, IMU and joints, 1 Hz temperatures, 0.5 Hz battery: sent to every address in the motion host's `network.toml` (X30 *Application Manual* 4.3.2); the driver listens on **43897** |
| perception host | `192.168.1.105`: elevation map on UDP **43899** (10 Hz feedback once it got any message), autonomous charging on UDP **3333** (request / response) |
| heartbeat | `0x31040001` at 5 Hz (at least 2 Hz), and `0x31020001` once to confirm the connection |
| command source | the first byte of every code: `0x31` an autonomous module (default, `command_prefix`), `0x21` a remote control terminal (the handheld) |
| velocity | axis commands `0x31010130` (+ forward), `0x31010131` (+ right), `0x31010135` (+ turn right) at 50 Hz, obeyed in the **stepping** state and **non-manual** mode |

**This computer must be in the motion host's `network.toml`** (its IP and port 43897, as the
Application Manual 4.3.2 describes; restart the motion program afterwards). Otherwise the robot
takes commands but sends no reports, and `state` says `no reports from 192.168.1.103 yet
(network.toml on the motion host?)`.

## Quick start

Ubuntu 20.04, ROS Noetic. Connect the computer to the robot's network and add it to
`network.toml` first (see [Network](#network)).

```bash
# 1. Sources, dependencies, build
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/DarmawanRobotics/x30_ros.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y
catkin_make && source devel/setup.bash

# 2. Launch: driver + URDF in the namespace x30
roslaunch x30_bringup bringup.launch

# 3. In a second terminal
rostopic echo -n 1 /x30/state
rosservice call /x30/stand_up
```

Without a robot, against the simulated one (motion and perception hosts on this computer):

```bash
rosrun x30_driver fake_robot.py                  # terminal 1: the simulated robot
roslaunch x30_bringup bringup.launch rviz:=true \
  overrides_file:=$(rospack find x30_driver)/config/fake_robot.yaml   # terminal 2
```

Then work through [Verify on the real robot](#verify-on-the-real-robot), including the `cmd_vel`
signs (step 6), before any autonomy drives the robot.

## Interface

Under the namespace (`/x30/` by default). Details in [x30_driver](x30_driver/README.md).

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | `base_link`, REP-103 (x forward, y left, z counter-clockwise); zeros after `cmd_timeout_sec`, then silence |
| pub | `odom` | `nav_msgs/Odometry` | leg odometry, `odom` → `base_link`; no TF unless `publish_tf: true` |
| pub | `imu` | `sensor_msgs/Imu` | attitude (reported in degrees), rates, acceleration; frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | `FL_HipX_joint` ... `HR_Knee_joint`: position, velocity, effort (torque) |
| pub | `battery` | `sensor_msgs/BatteryState` | `percentage` 0–1, voltage, current, capacity |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | `connected`, basic state, gait, control mode, errors, body height, temperatures, elevation map and charging state, ... as key / values |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | the same, plus link counters |

**Services** (`std_srvs/Trigger`; each waits until the robot reports the change):

| | |
|---|---|
| posture | `stand_up` (then torque control), `sit_down` (leaves stepping first), `torque_control`, `start_stepping`, `stop_stepping` |
| modes, height | `non_manual_mode`, `manual_mode`, `height_normal`, `height_crawl` |
| gaits (stepping state) | `gait_walk`, `gait_slope`, `gait_off_road`, `gait_stair`, `gait_stair_multi_frame`, `gait_stair_45`, `gait_l_walk`, `gait_mountain`, `gait_silent` |
| elevation map | `elevation_solid_ground`, `elevation_grid_ground`, `elevation_stairs_no_risers`, `obstacle_decelerate`, `obstacle_detour`, `velocity_source_joystick`, `velocity_source_navigation`, `obstacle_height_8cm`, `obstacle_height_28cm` |
| charging | `charge_start`, `charge_stop`, `charge_clear` |
| other | `emergency_stop` (soft emergency stop), `save_data` |

**Velocity.** `cmd_vel` goes out as axis values `v / vmax × 32767`, where `vmax` is the current
gait's maximum speed from Appendix B (forward, backward, lateral, yaw; half the forward speed for
walk and slope at crawl height). A non-zero command smaller than the dead zone (655) goes out as
656. Lateral and yaw are negated (the robot: + right, + turn right). Axis commands go out only in
the stepping state: in torque-controlled standing they would change the body's pose. The driver
switches to non-manual mode and starts stepping from torque-controlled standing when `cmd_vel`
asks for motion (`auto_non_manual_mode`, `auto_stepping`). `velocity_mode: topic` republishes
`cmd_vel` to `/cmd_vel_corrected` instead, for a driver on the X30's own ROS master (2.3).

## Verify on the real robot

On a new robot, in an open area, with someone at the handheld:

1. **Reports:** `rostopic hz /x30/odom` shows ~50 Hz and `/x30/state` says `connected: True`
   with the real basic state. If not: `network.toml` (above), then restart.
2. **Struct layouts:** `rcs_layout` and `motion_layout` in `state` read `natural`. A log line
   `report 0x... of N bytes: unknown layout` means the motion program packs a struct
   differently: report it with N.
3. **Command source:** with `command_prefix: 0x31` the robot obeys the services below and
   `errors` stays without `heartbeat_timeout`. If it ignores them, try `0x21` (as the handheld).
4. **Stand and step:** `rosservice call /x30/stand_up` ends in `torque_standing`,
   `/x30/start_stepping` in `stepping`; `sit_down` brings it back down.
5. **Forward:** `rostopic pub -r 20 /x30/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'`
   walks forward, `control_mode` turns `non_manual`, `odom` twist reads ~0.2. Stop the
   publisher: the robot stops within ~0.5 s.
6. **Turn and strafe (REP-103):** `{angular: {z: 0.3}}` must turn the robot **left**
   (counter-clockwise from above), else set `invert_wz: true`; `{linear: {y: 0.1}}` must move
   it **left**, else `invert_vy: true` (in your copy of
   [driver_params.yaml](x30_driver/config/driver_params.yaml), `params_file:=`).
7. **Speed scale:** in each gait you use, compare the `odom` twist with the command (Appendix B
   maximums; run and silent are not in the table and are scaled like walk).
8. **Battery:** `battery` voltage and current against the handheld app. The current is in 10 mA
   from Deeprcs 1.10.19 (`battery_current_scale: 0.01`), in A before (`1.0`);
   `battery_voltage_scale` if the voltage is off.
9. **Joints and frames:** `roslaunch x30_bringup bringup.launch rviz:=true`: the legs in RViz
   follow the real ones. `imu_link` sits at `base_link` (the document gives no IMU position);
   compare the LiDAR frames with the perception host's point cloud frames.
10. **Perception host:** `elevation_grid_ground` and back to `elevation_solid_ground` answer
    with the new mode; charging only with a dock in front of the robot.
11. **save_data:** set `save_data_code` to `18` for jy_exe older than v2.2.45 (it exits the
    motion program: try it last).

## Safety

- The driver stops on its own: no `cmd_vel` for 0.5 s gives zeros for 0.5 s, then nothing; the
  robot drops an axis command after 1 s. Only one node should publish `/x30/cmd_vel`.
- `emergency_stop` is the soft emergency stop: the robot lies down at once and protects its
  joints. The handheld and the robot's own stop button always work too.
- With `auto_non_manual_mode` the driver puts the robot in non-manual mode when motion starts:
  the handheld drives again only after `manual_mode` (or switching on the handheld).
- Axis commands only go out in the stepping state; `sit_down` refuses while the robot walks.
- Crawl height allows walk and slope only; L-Walk, Mountain and Silent allow no height change;
  the stair gaits need the matching elevation map mode. The services refuse otherwise.
- `save_data` makes the robot save its last 5 minutes of data and **exit its motion program**.

## Develop

```bash
catkin_make run_tests && catkin_test_results build/test_results --verbose
python3 -m pytest x30_driver/test x30_description/test   # the ROS-free tests, without ROS
```

The codec is tested against the document: its worked example (Appendix A), every command code it
prints, the struct sizes and field offsets of every layout, the gait speeds of Appendix B and the
velocity formulas of 1.2.3.2. The velocity policy, the UDP link and the simulated robot have
their own tests; a rostest runs the driver against the simulated robot through its ROS
interface (reports, posture, `cmd_vel` signs, gaits, height, elevation map, charging,
emergency stop). The URDF test checks the official model's numbers and that the joints are the
driver's. CI runs all of it in a `ros:noetic-ros-base` container, plus `check_urdf`.

The URDF and its meshes are Deep Robotics' model
([DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model),
BSD-3-Clause, [license](x30_description/LICENSE.deep_robotics_model)). x30_ros is released
under the [Apache License 2.0](LICENSE).
