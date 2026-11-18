# x30_perception

What the X30 sees around it, the ROS Noetic port of
[dharma_perception](https://github.com/DarmawanRobotics/dharma_perception): LiDAR obstacle
sectors for avoidance, a fail-safe 360° safety scan for the protective field of x30_control,
and a camera throttled for the fiducial detector.

## Nodes

| Node | Subscribes | Publishes |
|---|---|---|
| `lidar_sector_node` | `/livox/lidar` (the X30's Livox `CustomMsg`, raw bytes, default) or `input/pointcloud_topic` (PointCloud2, e.g. the X30's `/lidar_points`); TF sensor -> base, odom -> base | `perception/sector_scan` (front, avoidance), `perception/safety_scan` (360°, protective field), `perception/obstacles` (markers) |
| `fiducials_throttle` (`topic_tools/throttle`), `fiducials_info_relay` (`topic_tools/relay`) | `camera_image`, `camera_info` | `perception/fiducials/image` at `fiducials_rate`, every `perception/fiducials/camera_info` |
| `aruco_detector_node` | the throttled camera (`fiducials:=true`) | ArUco markers `ids` as TF frames `frames` under the camera frame (docking, localization) |

With `depth/enabled`, `lidar_sector_node` also subscribes a depth camera's PointCloud2
(`depth/topic`, see [Depth camera](#depth-camera)). Both scans are `x30_msgs/SectorScan` in
the robot profile's `frames/base`, stamped with the cloud time. `ranges[i] >= range_max` is
free; `ranges[i] <= SectorScan.BLOCKED` (0) means blocked or unknown and consumers treat it as
an obstacle at the robot. **Every sector is BLOCKED** when:

- the LiDAR is blind: fewer than `blind/min_points` points off the robot body, or (with
  `blind/min_region_points`) a quarter of the view is empty;
- the sensor mount TF (`frames/base <- sensor`) or, with `filter/level_source: tf`, the roll and
  pitch of the base (`frames/odom -> frames/base`) is missing;
- processing raised an exception, or no usable cloud arrived for `input/timeout_sec` (driver
  down, the wrong message type on the topic, only malformed messages);
- with `depth/required`, no usable depth cloud (see [Depth camera](#depth-camera)).

A malformed message is dropped with a throttled error; the node keeps running, and the launch
file respawns it anyway.

## On the X30

The X30's own sensor driver publishes the four LiDARs on `/livox/lidar` (and as a PointCloud2
on `/lidar_points`), on the motion and the navigation host's ROS master: run the stack on one of
them or point a payload computer's `ROS_MASTER_URI` at it. Two things to check before the robot
relies on the scans:

1. **The cloud's frame.** `rostopic echo -n 1 --noarr /livox/lidar/header` gives the
   `frame_id`; there must be a TF from `base_link` to it (the bringup's static transform) that
   matches where the driver puts the points. A wrong mount moves every obstacle.
2. **The leveling.** `filter/level_source: tf` needs `odom -> base_link` with the body's roll
   and pitch: x30_driver's leg odometry has them from the IMU (`publish_tf: true`), FAST-LIO too.

## Pipeline

1. **Parse.** Livox: the serialized ROS 1 CustomMsg is read with numpy (~1 ms per scan, no
   per-point Python objects, no Livox package installed here: the subscription takes the raw
   bytes and checks the publisher's type, `livox_ros_driver2/CustomMsg` or
   `livox_ros_driver/CustomMsg`). Every length is validated. PointCloud2:
   `x30_utils.ros.pointcloud.read_xyz`.
2. **Livox noise.** The tag byte marks points per field (bits 0-1 adhesion between objects, 2-3
   rain / fog / dust, 4-5 other; 0 high, 1 medium, 2 low confidence). Points at
   `filter/noise_level` (default: low) in the enabled fields are dropped.
3. **Self filter.** Points inside `robot/footprint` grown by `filter/self_margin`.
4. **Level.** Points in base_link are rotated by roll and pitch of the base in odom (TF at the
   cloud stamp), so the band is relative to the floor even while the gait pitches the body.
5. **Height band.** `filter.height_min..height_max` above the floor, with base_link
   `robot/base_height` above it. Optional `filter/ground_segmentation` estimates the floor per
   cell (lowest point within `ground_max_step`) for ramps and thresholds.
6. **Sectors.** Each sector's range is the `k`-th nearest point (`front/k`, `safety/k`, default
   2): a single dust return does not stop the robot; a sector with fewer than k points is free.
7. **Obstacle memory.** Confirmed safety points are kept in the odom frame for
   `memory/memory_sec` and merged into the safety scan while they are hidden below the lowest
   beam (`memory/lower_fov_deg`). An odometry jump clears the memory.
8. **Depth camera** (optional, below). Its obstacle points join both scans; a memory of its
   own keeps them beside the robot.

## Depth camera

A LiDAR mounted high and level sees low objects only far away (see [Limits](#limits)). A depth
camera pitched down sees the floor right ahead: `depth/enabled` merges its PointCloud2
(`depth/topic`, in any frame with TF to the base) into both scans. Every cloud is filtered as
it arrives:

1. **Range.** Points farther than `depth/range_max` from the camera are dropped: stereo depth
   error grows with the square of the range (a D435 at 424 × 240: ~1 cm at 1 m, ~7 cm at 3 m).
2. **Self filter, leveling, height band**: as the LiDAR's (`filter.*`).
3. **Floor fit.** The floor is fitted in the cloud (least squares on the points near the
   nominal floor, refined twice) and heights are measured above it, so a few degrees of error in
   the mount's pitch or roll, or the body pitching, never lift the far floor into the band:
   without it, 3° more pitch than the TF says lifts the floor 2 m ahead by 10 cm, a ghost
   obstacle. A fit with too few points, on a line (a wall fills the view), tilted more than
   `depth/floor_max_tilt_deg` or more than `depth/floor_max_offset` off the nominal floor under
   the robot keeps the nominal floor. Every minute the node logs the fit as the camera mount's
   offset from its TF (`pitch_deg`, `roll_deg`): a steady value is a mount to correct.
4. **Cells.** An obstacle needs `depth/cell_points` points in one `depth/cell_size` cell: a
   wrong stereo depth is a single point, a real object stacks its points above the floor.
5. **Confirmation.** And such a cell within `depth/confirm_radius` in the previous cloud too
   (at most 0.5 s before): a wrong depth lasts one frame, an object stays. A new obstacle is in
   the scans one camera frame later.
6. **Memory.** The camera loses a low box beside the robot as it passes (an L1W's D435i at
   about 0.3 m ahead of the body). Its obstacles stay in the odom frame for `depth/memory_sec`
   and join the safety scan while they are outside the camera's view (the azimuths and
   distances its cloud covers) and below the LiDAR's lowest beam; where the camera sees again,
   it forgets them, and a silent camera forgets nothing until they expire. Remembered points
   are 5 cm cell centres (`memory/resolution`), apart from the LiDAR's memory (`memory.*`).

The newest cloud's points join every LiDAR scan while it is younger than `depth/timeout_sec`
(one point makes a sector's range: the filters above did the confirming). A silent camera, a
blind one (fewer than `depth/min_points` points off the robot body: covered, blinded), a missing
camera TF or a malformed cloud leaves the scans to the LiDAR alone, with a throttled warning;
with `depth/required` they are BLOCKED instead. RViz: the `depth` namespace of
`perception/obstacles` shows the camera's own safety sector hits.

For a RealSense D400 (realsense2_camera for ROS 1): `filters:=pointcloud,decimation` with
`pointcloud_texture_stream:=RS2_STREAM_ANY` (no texture, so the whole depth field and not only
the part the color camera sees), a low `depth_width` / `depth_height`, and `clip_distance` a
little beyond `depth/range_max`. `cell_points` follows the density: 4 at 424 × 240 decimated
by 2.

## Limits

- **Near-field blind zone.** A Mid-360 sees down to -7°: mounted 0.53 m above the floor it sees
  a 10 cm object only beyond ~3.5 m. The memory keeps objects seen earlier, but an object that
  appears inside the zone (placed there, or the robot turned on next to it) is not seen. Use a
  pitched mount, a [depth camera](#depth-camera) or a near-field sensor (bumper, ToF, 2D
  scanner) where this matters. A depth camera has a blind zone of its own: right in front of
  the robot below its lowest ray, and beside the robot outside its field of view.
- **Negative obstacles** (stairs down, holes, platform edges) are not detected: missing floor is
  not an obstacle here.
- **Motion distortion.** The scan is not deskewed: a Mid-360 frame spans 100 ms, a point is up
  to `v * 0.1` m off (8 cm at 0.8 m/s, ~2° at 0.4 rad/s). Latency budget from first point to
  published scan: frame 100 ms + driver/transport ~5-10 ms + processing ~10-15 ms (20k points, one desktop core; measure on the target); keep
  `robot/limits/reaction_time` above it (0.25 s default) and the protective field margins
  above the distortion.
- **Sector quantization.** The nearest point of a sector stands for the whole sector:
  72 safety sectors are 5° wide (17 cm at 2 m); raise `safety/num_sectors` for finer fields.
- This is a software safety channel. It does not replace a safety-rated scanner or a hardware
  e-stop (see x30_control).

## Parameters

Private parameters of `lidar_sector_node`, read-only at runtime (a change on the server is
refused with a warning until a restart) and validated at start. Every one is documented in
[core/config.py](src/x30_perception/core/config.py); the important ones are in
[config/lidar.yaml](config/lidar.yaml). ArUco: [config/aruco.yaml](config/aruco.yaml). The
robot (frames, footprint, `base_height`) comes from the robot profile (`robot/`).

## Launch

```bash
roslaunch x30_perception perception.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rostopic echo -n 1 /perception/safety_scan
roslaunch x30_perception perception.launch fiducials:=true camera_image:=/camera/image_rect camera_info:=/camera/camera_info
```

Arguments: `params_file` (overrides), `robot_params` (loaded as `robot/`; empty: already on the
server), `fiducials`, `fiducials_rate` (5 Hz), `fiducials_params`, `camera_image`,
`camera_info`, `respawn`, `output`.

## Test

```bash
catkin_make run_tests_x30_perception && catkin_test_results build/test_results/x30_perception
```

The pipeline, the Livox parser (layout by hand, fuzzing, parity with genpy's serializer), the
depth filter and the ArUco pose are tested without ROS; the nodes run against a private master
(Livox bytes and the topic with its type check, noise tags, blind and malformed input, unknown
frames, a pitched body, PointCloud2, the watchdog, invalid parameters, the depth camera, ArUco
over `sensor_msgs/CameraInfo` to TF).

x30_perception is released under the [Apache License 2.0](../LICENSE).
