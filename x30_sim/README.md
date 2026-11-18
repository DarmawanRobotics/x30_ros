# x30_sim

The X30 in software for the whole stack, the ROS Noetic port of
[dharma_sim](https://github.com/DarmawanRobotics/dharma_sim): a kinematic body behind
x30_driver's topics and services, the X30's four Livox Mid-360 ray-cast into the real
`/livox/lidar` type, the body IMU, a drifting FAST-LIO stand-in and leg odometry, latency and
dropouts from seeded scenarios, worlds and ready sites for end-to-end tests. No Gazebo, no GPU.

`roslaunch x30_bringup bringup.launch sim:=true ...` runs the stack on it exactly as on the robot
(the odometry layer with `odometry_method:=external`); x30_bringup's `stack_sim.test` drives a
route through it in CI.

## Node

`sim_robot_node` stands in for x30_driver, the LiDARs, the IMU and FAST-LIO:

| On the X30 | In the simulator |
|---|---|
| x30_driver (`/x30/`, `driver_ns`) | subscribes `cmd_vel`; publishes `odom` (leg odometry: pose, twist in `base_link`, the driver's covariances; TF `odom -> base_link` only with `publish_tf`) and `battery` (`percentage`; status UNKNOWN like the driver, unless `battery/report_charging`); services `sit_down` (refused while walking), `stand_up` (refused while charging), `charge_start` (sitting on the world's dock contacts), `charge_stop` |
| four Mid-360, merged | `/livox/lidar`: `livox_ros_driver2/CustomMsg` bytes (the driver's md5, read by x30_perception and FAST-LIO alike), points in `base_link`, frame `livox_frame` |
| body IMU | `/imu/data` in `imu_link`: specific force (m/s², gravity, linear, centripetal and tangential acceleration at its origin) and rates |
| FAST-LIO2 | per scan, with the scan's stamp: `/Odometry` and TF `camera_init -> body` (the IMU, no twist, as FAST-LIO), `/cloud_registered_body` (the scan in `body`, nothing within `lio/blind` of `base_link`) |
| a tag in view | `/initialpose` at the true pose while `localization/status` is `NONE` or not yet published |
| inspection at a stop point | `sim/inspect` (`std_srvs/Trigger`) answers `clear` |
| camera + tag detector | TF `camera_color_optical_frame -> <tag>` (`camera/frame`, x30_localization's `tag/camera_frame`) for the world's `camera/tags` in view, in range and not hidden by a wall: localization's tag corrections and docking; the mount `base_link -> camera_color_optical_frame` on `/tf_static` (`camera/publish_mount`, off once the URDF has the camera) |
| people and boxes | `sim/obstacles` (`geometry_msgs/PoseArray`, map frame) replaces the dynamic circles |
| the truth | `sim/ground_truth` (PoseStamped, `map`), `sim/collision` (Bool, latched; true on contact, the body stops there) |

The URDF transforms (`base_link -> imu_link`, ...) come from robot_state_publisher as on the
robot; x30_bringup's `lidar_mount` keeps `livox_frame` on `base_link`.

## Models

All in `x30_sim/core` (pure Python + numpy, unit-tested):

| Module | |
|---|---|
| `world.py` | walls (segments) and pillars / people / boxes (circles), each with a height; ray casting (also from inside a circle) and footprint collision queries |
| `body.py` | omni or diff body (`robot/kinematics`), `robot/limits` accelerations, integrates the measured dt (clamped to `max_dt`), keeps its acceleration, stops and latches on contact |
| `lidar.py` | the four Mid-360 of application manual 6.2: each a ring of `rings` × `azimuth_rays` around its dome (vertical fov −7°..52°), tilted (front 25° up and 46° down, rear 5° and 56° down), cast from its height in the 2.5D world, floor returns, range noise, dropout; merged in `base_link` |
| `odometry.py` | drift `T_odom_base <- T_odom_base * xi`, `xi ~ N(dT, Sigma ds)` plus scale error and yaw bias, for FAST-LIO (`odometry/`) and the legs (`leg_odometry/`); FAST-LIO's pose through the IMU offset |
| `latency.py` | stamps `t - L` showing the state of that time (pose history); commands act `L` after they arrive, zero once stale |
| `imu.py` | specific force and rates at the IMU's origin on the level body |
| `scenario.py` | seeded scenario files (below) |
| `dock.py` | tag camera (field of view, range, incidence, occlusion by walls, noise) and the dock station (sit, stand, charge on the contacts) |
| `simulation.py` | everything above on one clock |
| `site_fixture.py` | a site from a world: `map.pgm` + `map.yaml` (map_server trinary, size and CRC), `tags.json`, `routes/*.csv` (x30_utils.io) |

The body is planar: no gait, pitch or roll, no ceiling, no self-occlusion (rays through the body
land under it, where x30_perception's footprint filter and FAST-LIO's blind drop them anyway).
Sixteen rings sample the Mid-360's non-repetitive pattern: the floor ahead is dense to 2.25 m,
then one ring every few metres.

## Scenarios

[config/scenarios](config/scenarios) (YAML; times in seconds since start):

```yaml
seed: 3 # replaces the seed parameter: the same file gives the same run
latency: { lidar_sec: 0.15, odometry_sec: 0.05, command_sec: 0.05 }
odometry: { sigma_x: 0.01, scale_error: 0.01, yaw_bias_deg_per_m: 0.1 } # FAST-LIO
leg_odometry: { scale_error: 0.03 }
obstacles: # circles from start to end (default: forever), moving with vx, vy
  - { start: 8.0, end: 30.0, x: -1.5, y: -4.2, vy: 0.15, radius: 0.25, height: 1.7 }
dropouts: # sensor: lidar (FAST-LIO too) | odometry (FAST-LIO) | leg_odometry | imu | all
  - { start: 10.0, duration: 1.5, sensor: lidar }
odometry_restarts: [40.0] # FAST-LIO restarts: camera_init at the current pose
```

| Scenario | |
|---|---|
| `drift` | 1 % scale error, yaw bias and noise: localization corrections |
| `person_crossing` | a person crosses the corridor, a box stays on the route |
| `sensor_faults` | LiDAR, odometry and full dropouts, an odometry restart, late stamps |

## Worlds and sites

[config/worlds/mall_loop.yaml](config/worlds/mall_loop.yaml): a 12 × 9 m mall, a block of shops
in the middle, a 3 m corridor around it, three pillars, the `base_map` and `charging_dock` tags
and the dock (base_link 1.0 m in front of the dock tag, x30_navigation's `goal/distance`).
[config/routes/mall_loop.csv](config/routes/mall_loop.csv) is a taught loop with one stop point.

`draw_site` writes a site from a world (map, tags, the world's route), unless it has a map:

```bash
rosrun x30_sim draw_site --site mall --sites-dir /tmp/x30_sim/sites   # prints the site path
```

`sim.launch` runs it while the launch file is read (`site:=`), so the site exists before the
mapping and localization nodes load it.

## Parameters

Defaults: [config/sim.yaml](config/sim.yaml); every parameter with its unit and range:
[src/x30_sim/core/config.py](src/x30_sim/core/config.py). The robot (kinematics, footprint,
base_height, limits, frames) comes from the robot profile (`robot/`).

| Parameter | Default | |
|---|---|---|
| `world_file` / `scenario_file` | `''` / `''` | path or name in `config/worlds` (`''`: mall_loop), `config/scenarios` |
| `driver_ns` | `x30` | x30_driver's topics and services |
| `seed` | `0` | RNG seed (a scenario's seed wins) |
| `rate_hz` / `odom_rate_hz` / `imu/rate_hz` | `100` / `50` / `200` | physics, leg odometry, IMU (0: off) |
| `max_dt` / `cmd_timeout_sec` | `0.1` / `0.5` | longest physics step, stale command |
| `publish_tf` | `false` | `odom -> base_link` from the leg odometry (x30_bringup `leg_odom_tf`) |
| `lidar/rate_hz` / `rings` / `azimuth_rays` | `10` / `16` / `360` | per LiDAR |
| `lidar/mounts` | application manual 6.2 | per LiDAR: x y z (m, `base_link`), dome azimuth and elevation (deg) |
| `lidar/range_min` / `range_max` / `noise_m` | `0.1` / `40` / `0.01` | m |
| `lio/blind` | `0.7` | m around `base_link`, as FAST-LIO's |
| `imu/xyz` | `0.13045 -0.001 0.0149` | `imu_link` in `base_link` (the URDF, FAST-LIO's `extrinsic_T` negated) |
| `odometry/*`, `leg_odometry/*` | `0.005` / `0.02` m/√m … | `sigma_x`, `sigma_y`, `sigma_yaw_deg`, `scale_error`, `yaw_bias_deg_per_m` |
| `latency/lidar_sec` / `odometry_sec` / `command_sec` | `0.05` / `0.01` / `0.02` | s |
| `obstacle_radius` / `obstacle_height` | `0.25` / `1.7` | `sim/obstacles` circles (live) |
| `initial_pose_hz` | `1.0` | `/initialpose` while NONE (0: off) |
| `camera/rate_hz` / `tags` / `mount_xyz` / `range_max` | `10` / `[base_map, charging_dock]` / `0.5 0 0.1` / `4.0` | tag camera (0 Hz: off), `yaw_deg: 180`: rear |
| `battery/start` / `drain_per_hour` / `charge_per_hour` | `0.9` / `0.2` / `1.0` | |

## Launch

```bash
roslaunch x30_sim sim.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml site:=mall
roslaunch x30_sim sim.launch scenario:=person_crossing
# a person steps onto the route
rostopic pub -1 /sim/obstacles geometry_msgs/PoseArray "{poses: [{position: {x: 2.0, y: -3.0}}]}"
```

| Argument | Default | |
|---|---|---|
| `params_file` | | parameters on top of `config/sim.yaml` |
| `robot_params` | | the robot profile, loaded as `robot/` (x30_bringup loads it itself) |
| `world` / `scenario` | | file or name (empty: from the files) |
| `site` / `sites_dir` | / `/tmp/x30_sim/sites` | draw this site from the world unless it has a map |

## Test

`test/unit` (the models, the Livox bytes against x30_perception's parser, the IMU against the
URDF and FAST-LIO's extrinsic, the LiDAR mounts against the application manual),
`test/ros` (the node against a private master, the launch file, `draw_site`),
`test/test_style.py`.
