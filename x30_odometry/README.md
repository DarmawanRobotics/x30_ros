# x30_odometry

The LiDAR-inertial odometry of the X30, the ROS Noetic port of
[dharma_odometry](https://github.com/DarmawanRobotics/dharma_odometry): FAST-LIO2 on the X30's
merged LiDARs and body IMU, a start gate that waits for a still and level IMU, and a monitor
that says when the odometry can no longer be trusted and how far the robot went.

FAST-LIO publishes `camera_init -> body` and `/Odometry`, starting wherever the robot is at
launch. It drifts slowly; x30_localization keeps `map -> odom` on top of it and bridges it into
`odom -> base_link`.

## Nodes

`odometry.launch` starts:

| Part | Role |
|---|---|
| `fast_lio` (in `fast_lio/`) | `method:=fast_lio` (default): FAST-LIO2 (`fast_lio/fastlio_mapping`, ROS 1) with [config/fast_lio.yaml](config/fast_lio.yaml); `method:=external` when a simulator or another estimator publishes `/Odometry` |
| `odometry_gate` (`wait_still`) | the launch prefix of FAST-LIO: holds its start, and every respawn, until the IMU is still and level (at most `max_tilt_deg`, 3°), since FAST-LIO takes gravity and the IMU biases from its first samples |
| `odometry_monitor_node` | `odometry/status`, `odometry/restarted` ([Monitor](#monitor)) and `odometry/trip` ([Trip](#trip)) |

| Interface | Type | |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO, `camera_init -> body` (input) |
| `/x30/odom` (`leg_odom_topic`) | `nav_msgs/Odometry` | x30_driver's leg odometry for the speed test, optional (input) |
| `/imu/data` (`imu_topic`) | `sensor_msgs/Imu` | the X30's body IMU: stillness and tilt (input) |
| `odometry/status` | `x30_msgs/OdometryStatus`, latched | `state` (`INITIALIZING`, `OK`, `DEGRADED`, `LOST`), `speed` m/s, `tilt` rad, `message` |
| `odometry/restarted` | `std_msgs/Empty` | the odom frame was reset: `map -> odom` must be estimated again |
| `odometry/trip` | `x30_msgs/TripStatus`, latched | distance, moving time, speed now, average and top speed since the last reset |
| `odometry/trip_reset` | `std_srvs/Trigger` (service) | start the trip again from zero |

## Monitor

| State | When |
|---|---|
| `INITIALIZING` | no `/Odometry` yet (the gate is holding FAST-LIO) |
| `LOST` | no `/Odometry` for `stale_sec`; a jump (a step faster than `jump/speed_factor` × the robot speed limit `robot/limits`) or a restart, for `hold_sec` |
| `DEGRADED` | \|z\| above `max_z_drift_m` on a flat floor; odom tilted against gravity by more than `tilt/max_deg` while standing still; the speed test in alarm |
| `OK` | otherwise |

- **Restart.** A fresh estimator starts at the identity pose. A restart is a pose back at the
  origin after a message gap of `restart/gap_sec` (respawn, the gate) or after a translation
  jump the robot cannot have made, or a stamp going back. Yaw alone never counts: turning in
  place next to the start pose is not a restart.
- **Speed test.** `r = v_lio − v_leg` (planar speeds), `χ² = r² / (σ_lio² + σ_leg²)`, one-sided
  CUSUM `s = max(0, s + χ² − κ)`, alarm while `s > h` (`speed/sigma_*`, `speed/cusum_drift`,
  `speed/cusum_threshold`); no per-robot speed thresholds. Off without `leg_odom_topic` or
  while it is silent.
- **Tilt.** While the IMU is still, `tilt` is the roll and pitch of the body in odom. The
  gravity measured by the IMU, rotated into odom, gives the tilt of odom itself: above
  `tilt/max_deg` FAST-LIO started tilted and z drifts with distance. A robot standing on a ramp
  tilts its body, not odom, and stays `OK`. The X30's `/imu/data` has the body's axes and is
  FAST-LIO's IMU, so its axes are the `/Odometry` body axes.

## Trip

`odometry/trip` counts how far the robot went and how fast since the last reset
(`rosservice call /odometry/trip_reset`), from the poses of `/Odometry`, so it also counts while
nobody watches. The totals are written to `trip/file` (`~/.x30/trip.json`, at most every
`trip/save_sec`, at once on a reset and at shutdown) and continue after a restart.

- **Distance** is planar and grows by whole steps of `trip/step_m` (5 cm): the pose noise of a
  standing robot adds nothing.
- **Moving time** is the time between two counted steps covered at `trip/moving_mps` or faster;
  **average** is distance / moving time.
- **Speed** is the straight distance over the last `trip/window_sec`; **top speed** its highest
  value.
- Nothing is counted across a gap: silence longer than `trip/gap_sec`, a step faster than
  `trip/max_mps`, a jump or a restart.

## Parameters

Defaults in the dataclasses ([core/config.py](src/x30_odometry/core/config.py)), the X30's
values in [config/monitor.yaml](config/monitor.yaml) and [config/gate.yaml](config/gate.yaml).
They are validated at start (a node with an invalid value exits with code 2); the monitor
applies live changes on the parameter server within a second, all or none.

<details>
<summary><b>odometry_gate (wait_still)</b>, in <code>fast_lio/odometry_gate/</code></summary>

| Parameter | Default | |
|---|---|---|
| `imu_topic` | `/imu/data` | IMU topic |
| `still_sec` | `2.0` | this long without motion before the start [s] |
| `max_gyro_rad_s` | `0.03` | largest angular rate in the window [rad/s] |
| `max_acc_rel_std` | `0.01` | std of \|acc\| / mean \|acc\| (unit-free: m/s² or g) |
| `max_tilt_deg` | `3.0` | gravity against the IMU z axis; 0: off [deg] |
| `min_samples` | `20` | IMU samples needed in the window |
| `timeout_sec` | `60.0` | start anyway (with an error); 0: wait forever [s] |

</details>

<details>
<summary><b>odometry_monitor_node</b></summary>

| Parameter | Default | |
|---|---|---|
| `odom_topic` | `/Odometry` | estimator odometry |
| `leg_odom_topic` | `/x30/odom` | leg odometry for the speed test; empty: off |
| `imu_topic` | `/imu/data` | IMU with the axes of the odometry body; empty: no tilt check |
| `status_period_sec` | `0.2` | odometry/status period [s] |
| `stale_sec` | `0.5` | no odometry this long: LOST [s] |
| `hold_sec` | `5.0` | a jump or restart stays LOST this long [s] |
| `max_z_drift_m` | `0.5` | \|z\| above this is DEGRADED (flat floor); 0: off [m] |
| `restart/gap_sec`, `restart/origin_radius_m`, `restart/origin_yaw_deg` | `2.0`, `0.05`, `1.0` | a gap this long followed by a pose this close to the origin is a restart |
| `jump/speed_factor`, `jump/min_step_m` | `3.0`, `0.3` | a step faster than this × the speed limit and longer than this is a jump |
| `speed/window_sec`, `speed/sigma_lio_mps`, `speed/sigma_leg_mps` | `0.5`, `0.05`, `0.15` | the speed test |
| `speed/cusum_drift`, `speed/cusum_threshold`, `speed/leg_timeout_sec` | `3.84`, `20.0`, `0.3` | κ, h, older leg odometry is not compared |
| `tilt/max_deg` | `5.0` | odom z against gravity above this; 0: off [deg] |
| `tilt/still_sec`, `tilt/max_gyro_rad_s`, `tilt/max_acc_rel_std`, `tilt/min_samples` | `1.0`, `0.03`, `0.01`, `20` | when the IMU counts as still for the tilt |
| `trip/enabled`, `trip/file` | `true`, `~/.x30/trip.json` | the trip; empty file: not kept |
| `trip/step_m`, `trip/window_sec`, `trip/moving_mps` | `0.05`, `1.0`, `0.05` | |
| `trip/gap_sec`, `trip/max_mps`, `trip/period_sec`, `trip/save_sec` | `1.0`, `3.0`, `0.5`, `10.0` | |

</details>

## Launch

```bash
roslaunch x30_odometry odometry.launch                     # FAST-LIO2 behind the gate
roslaunch x30_odometry odometry.launch save_map:=true      # keep the map, PCD at shutdown
roslaunch x30_odometry odometry.launch method:=external    # simulator: the monitor only
rostopic echo /odometry/status
```

| Argument | Default | |
|---|---|---|
| `method` | `fast_lio` | `fast_lio` or `external`; anything else fails the launch |
| `wait_still` | `true` | hold FAST-LIO (and every respawn) until the IMU is still |
| `save_map` | `false` | FAST-LIO keeps its map for a PCD; RAM grows |
| `params_file` | | overrides of config/monitor.yaml |
| `gate_params_file` | | overrides of config/gate.yaml |
| `fast_lio_params_file` | config/fast_lio.yaml | another tuning |
| `leg_odom_topic` | | overrides `leg_odom_topic` of the monitor |
| `robot_params` | | the robot profile, loaded as `robot/` |
| `respawn`, `output` | `true`, `screen` | FAST-LIO respawns behind the gate; the monitor always respawns |

FAST-LIO reads its parameters from its namespace, not its private one, so the launch keeps
them in `fast_lio/`; its topics (`/Odometry`, `/cloud_registered_body`) are absolute.

## FAST-LIO on the X30

FAST-LIO2 for ROS 1 (hku-mars `FAST_LIO`, built from source: it is not released for Noetic) on:

- `/livox/lidar`, the four Mid-360 of the X30 merged by its driver (`livox_ros_driver2/CustomMsg`,
  10 Hz). FAST-LIO subscribes with `livox_ros_driver/CustomMsg`: both have the same definition and
  md5, which is all ROS 1 compares, so either Livox message package satisfies its build.
- `/imu/data`, the body IMU (200 Hz) at (130.45, −1, 14.9) mm in the body frame with the body's
  axes (Application Manual V2.0.4, 6.2); x30_description's `imu_link` is there.

### Check on the robot

[config/fast_lio.yaml](config/fast_lio.yaml) takes the points of `/livox/lidar` to be in the
body frame (`base_link`), so FAST-LIO's LiDAR pose in the IMU frame is minus the IMU mount
(`extrinsic_T: [-0.13045, 0.001, -0.0149]`, no rotation). Check it before the first map:

1. `rostopic echo -n 1 --noarr /livox/lidar/header` and the `lidar_id` of a few messages: one
   frame for all four LiDARs.
2. With the robot standing on a flat floor, the floor of `/lidar_points` in RViz (fixed frame:
   that frame) is level and about `base_height` (0.48 m) below the origin; the front LiDARs'
   points start ahead of x = 0.44 m. A cloud in a LiDAR's own frame is tilted by its mount (25°
   to 56°) and needs that mount as `extrinsic_T`/`extrinsic_R` instead.
3. `blind` (0.7 m from the frame origin) hides the body and the legs; with the robot standing,
   `/cloud_registered_body` must show none of them.

## Tests

`catkin_make run_tests_x30_odometry`: the core (unit), the monitor and the gate against a private
rosmaster (restart, turn in place, trip kept across a restart and reset, live parameters, the
gate releasing or holding, the exec after a timeout, SIGINT/SIGTERM while held, invalid
parameters), and the launch file's parameters for every method.
