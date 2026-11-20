# x30_localization

Where the X30 is on its site, the ROS Noetic port of
[dharma_localization](https://github.com/DarmawanRobotics/dharma_localization) without a map:
`map -> odom` from an EKF on top of FAST-LIO, corrected by fiducials in view and operator pose
estimates, through a gate that grows with the uncertainty.

The map frame is where the odometry started (`assume_start_origin`: the dock, every time); the
site's tags were recorded in it, so a tag in view puts the robot back on that frame wherever it
is, after an odometry restart too. Any marker detector works as long as it puts the marker
frames under the camera optical frame on TF (x30_perception's `aruco_detector_node`,
`apriltag_ros`).

```
map -> odom -> camera_init -> body -> base_link
```

## Node

`localization_node` bridges FAST-LIO into this tree with two static transforms:
`odom -> camera_init = L · T_base_imu` and `body -> base_link = T_base_imu⁻¹`, where
`T_base_imu` is `base_link -> imu_link` of x30_description (the body IMU of FAST-LIO) and `L` is
the roll and pitch of the base at the odometry start, measured by the IMU while standing still.
So `odom` is **gravity-aligned** (z up, x the start heading) and starts at `base_link`. After
`odometry/restarted` it is leveled again for the new start. `map -> odom` is broadcast on `/tf`
at `publish_hz`.

| Interface | Type | |
|---|---|---|
| `localization/start` | `std_srvs/Trigger` | localize from a tag in view, **explicitly** (no gate) |
| `localization/reset` | `std_srvs/Trigger` | forget `map -> odom`; tags in view localize again |
| `localization/assume_origin` | `std_srvs/Trigger` | `map -> odom` = identity: the odometry origin is the map origin (FAST-LIO run since the start spot) |
| `localization/record_tags` | `std_srvs/Trigger` | average the map pose of the visible `tag/frames` and merge them into the file |
| `localization/load_tags` | `x30_msgs/LoadFile` | switch the tags file (inside `sites_dir`); `create` starts an empty one |
| `localization/status` | `x30_msgs/LocalizationStatus`, latched | `NONE` / `LOCALIZED` / `UNCERTAIN`, source, sigma_xy, sigma_yaw |
| `localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map -> base_link` with its planar covariance, while localized |
| `localization/tag_markers` | `visualization_msgs/MarkerArray`, latched | the known tags |
| `odometry/restarted` | `std_msgs/Empty` | the odometry started over: `map -> odom` and the level are void |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D Pose Estimate (operator, trusted, no gate) |
| `/imu/data` | `sensor_msgs/Imu` | leveling (`leveling/imu_topic`) |

`record_tags` writes site anchors, so it refuses unless the state is `LOCALIZED`, and it never
records a tag the current `map -> odom` was derived from (circular). It runs beside the node's
timers (`create_service(..., concurrent=True)`): `map -> odom` keeps flowing while it records.
A corrupt tags file is reported (`success: false`) and left untouched; a `.bak` is kept on every
write.

<details>
<summary><b>Parameters</b></summary>

Frames and `base_height` come from the robot profile (`robot/`). Defaults in
[core/config.py](src/x30_localization/core/config.py), the X30's in
[config/localization.yaml](config/localization.yaml).

| Parameter | Default | |
|---|---|---|
| `sites_dir` | `~/x30/map` | `load_tags` paths must resolve inside |
| `tags_file` | `""` | tags file at start (`~` and `$VAR` expanded) |
| `lio_odom_frame`, `lio_body_frame`, `imu_frame` | `camera_init`, `body`, `imu_link` | the bridge |
| `publish_hz`, `pose_hz` | `20`, `10` | `map -> odom`, `localization/pose` |
| `assume_start_origin` | `true` | the odometry origin is the map origin once the bridge is up (never after an odometry restart) |
| `estimator/sigma_xy_per_sqrt_m` | `0.03` | drift per metre travelled (not per second) |
| `estimator/sigma_yaw_deg_per_sqrt_m`, `..._per_sqrt_rad` | `0.1`, `0.5` | heading drift per metre and per radian turned |
| `estimator/gate_confidence` | `0.99` | chi-square gate of automatic corrections (3 dof) |
| `estimator/max_sigma_xy`, `max_sigma_yaw_deg` | `100`, `180` | above: `UNCERTAIN`; only tags correct the drift, so it stays `LOCALIZED` and sigma reports it |
| `tag/camera_frame`, `tag/frames` | `camera_color_optical_frame`, `[base_map, charging_dock]` | |
| `tag/auto_localize`, `tag/auto_correct` | `true`, `true` | tags in view localize / correct through the gate |
| `tag/max_distance_m`, `tag/max_view_angle_deg` | `4.0`, `60` | farther or more oblique detections are ignored |
| `tag/sigma_xy_at_1m`, `tag/sigma_yaw_deg_at_1m` | `0.02`, `1.0` | detection noise, growing with d² (position) and d (yaw) |
| `tag/record_duration_sec`, `record_min_samples` | `2.0`, `5` | `record_tags`; noisier recordings are refused |
| `leveling/enabled`, `leveling/imu_topic` | `true`, `/imu/data` | gravity alignment of odom |
| `leveling/timeout_sec` | `15` | then the bridge goes up unleveled (warning) |

</details>

## Estimator

An SE(2) Kalman filter over `map -> odom` with a 3×3 covariance:

- **predict** adds noise per metre travelled and per radian turned by the odometry (counted in
  steps, so a robot standing still never grows uncertain);
- **update** takes a measurement of `map -> base_link` with its covariance; the innovation is on
  `map -> base_link`, so a fix far from the odometry origin is judged by how far it moves the
  robot, and a small rotation that would swing the robot metres away is rejected;
- automatic sources pass a **chi-square gate** on the innovation (Mahalanobis distance against
  `H P Hᵀ + R`): the gate opens as the uncertainty grows after a long walk and stays tight right
  after a correction; the covariance update is in Joseph form;
- tag noise grows with distance and view angle, and a heading error moves the robot around the
  tag (lever arm), so far tags count little.

`auto_correct` uses the gate; `localization/start`, a pose estimate and `assume_origin` are
explicit and set `map -> odom` directly. A far but consistent tag that the gate rejects (after a
kidnapping, a wrong 2D Pose Estimate) therefore still relocalizes through `localization/start`.

A correction is published as the previous `map -> odom` 1 ms before the new one, so tf2 never
interpolates a ramp across it.

## Launch

```bash
roslaunch x30_localization localization.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rosservice call /localization/load_tags "{path: site/tags.json, create: true}"
rosservice call /localization/start
rostopic echo /localization/status
```

Arguments: `params_file`, `robot_params`, `tags_file`, `sites_dir` (empty ones override nothing),
`respawn`, `output`.

## Tests

`catkin_make run_tests_x30_localization`: the estimator, the leveling and the bridge and tags
(unit), and the node against a fake X30 (FAST-LIO TF, a tilted IMU, a tag
detector) on a private rosmaster: the leveled bridge, record_tags while `map -> odom` keeps
flowing, relocalizing after an odometry restart, the tags file path guard and a corrupt file.
