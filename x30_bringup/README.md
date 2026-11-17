# x30_bringup

The Jueying X30 in one launch: [x30_driver](../x30_driver/README.md) and
`robot_state_publisher` with the URDF of [x30_description](../x30_description/README.md) in one
namespace, the robot profile every node of the stack reads, and optionally the control mux
([x30_control](../x30_control/README.md)), the LiDAR sector scans
([x30_perception](../x30_perception/README.md)), FAST-LIO and its monitor
([x30_odometry](../x30_odometry/README.md)), the localization
([x30_localization](../x30_localization/README.md)), the site maps
([x30_mapping](../x30_mapping/README.md)) and the navigation
([x30_navigation](../x30_navigation/README.md)).

## Usage

```bash
roslaunch x30_bringup bringup.launch                                  # namespace x30
roslaunch x30_bringup bringup.launch params_file:=$HOME/x30_driver_params.yaml rviz:=true
roslaunch x30_bringup bringup.launch control:=true perception:=true leg_odom_tf:=true
roslaunch x30_bringup bringup.launch control:=true perception:=true \
    odometry:=true localization:=true mapping:=true site:=gardu-a navigation:=true
```

With `control:=true` the control mux is the only publisher of `/x30/cmd_vel`: navigation
publishes `control/nav_cmd_vel`, an operator `control/remote/<source>/cmd_vel`, and nothing
else may publish the driver's `cmd_vel` any more.

## Launch

`launch/bringup.launch`:

| Argument | Default | |
|---|---|---|
| `namespace` | `x30` | driver topics and services, `robot_description` |
| `params_file` | [`x30_driver/config/driver_params.yaml`](../x30_driver/config/driver_params.yaml) | a copy with this robot's settings, e.g. `invert_wz` |
| `overrides_file` | empty | a few parameters on top, e.g. `x30_driver/config/fake_robot.yaml` |
| `robot_params` | [`config/robot.yaml`](config/robot.yaml) | the robot profile, loaded as `/robot` |
| `control` | `false` | the control mux (x30_control), publishing `/<namespace>/cmd_vel` |
| `perception` | `false` | `lidar_sector_node` (x30_perception) and the LiDAR mount TF |
| `leg_odom_tf` | `false` | the driver publishes `odom` → `base_link` (leg odometry, roll and pitch from the IMU): the scans' leveling, while no other odometry owns that transform; refused together with `localization` |
| `odometry`, `odometry_method` | `false`, `fast_lio` | FAST-LIO behind its start gate and the odometry monitor (x30_odometry); `external` for a simulator |
| `localization` | `false` | `map` → `odom` and the bridge of FAST-LIO into `odom` → `base_link` (x30_localization) |
| `mapping`, `site` | `false`, empty | the site maps, mapping sessions and routes (x30_mapping); the site at start |
| `navigation` | `false` | the planner, route loader, path follower, avoidance and docking (x30_navigation), commanding the control mux; the robot's battery and odometry under `namespace` |
| `lidar_frame`, `lidar_pose` | `livox_frame`, `0 0 0 0 0 0` | `base_link` → the frame of `/livox/lidar` (`x y z yaw pitch roll`, m and rad) |
| `rviz` | `false` | RViz with the robot model |

## Nodes

| Node | Namespace | |
|---|---|---|
| `x30_driver` | `x30` | `cmd_vel` in; `odom`, `imu`, `joint_states`, `battery`, `state`, `diagnostics`; Trigger services |
| `robot_state_publisher` | `x30` | TF `base_link` → `TORSO` → legs, `imu_link`, `LIDAR_*` from `joint_states` |
| `control_mux_node` (`control:=true`) | `/` | `control/*` in, `/<namespace>/cmd_vel` out |
| `lidar_sector_node`, `lidar_mount` (`perception:=true`) | `/` | `/livox/lidar` in, `perception/sector_scan`, `perception/safety_scan` out |
| `fast_lio` (in `fast_lio/`), `odometry_monitor_node` (`odometry:=true`) | `/` | `/livox/lidar`, `/imu/data` in; `/Odometry`, `camera_init` → `body`, `odometry/*` out |
| `localization_node` (`localization:=true`) | `/` | `map` → `odom`, `odom` → `camera_init`, `body` → `base_link`; `localization/*` |
| `grid_mapper_node`, `site_manager_node`, `route_recorder_node` (`mapping:=true`) | `/` | `/map`, `mapping/*`, `site/*`, `route/*` |
| `planner_node`, `path_loader_node`, `path_follower_node`, `obstacle_avoidance_node`, `docking_node` (`navigation:=true`) | `/` | `planner/*`, `navigation/*`, `docking/*`; `control/nav_cmd_vel` out |
| `rviz` (`rviz:=true`) | `x30` | |

The driver publishes no TF by default (`publish_tf: false`): `odom` → `base_link` belongs to
the odometry or localization in use; `leg_odom_tf:=true` hands it to the driver. With
`localization:=true` the tree is `map` → `odom` → `camera_init` → `body` → `base_link`, and the
launch refuses `leg_odom_tf:=true` (`base_link` would have two parents).

## Robot profile

[config/robot.yaml](config/robot.yaml) is the X30 for the stack: footprint (standing 1000 ×
470 mm around the body centre), `base_height`, motion limits and frames. Its limits are what the
stack may command: they are reachable in the walk gait (and in L-Walk, which is faster), so the
driver never clips a command the stack plans with. A test checks the file against the robot
profile and against appendix B of the API documentation.

Before the robot drives on its own, measure and set:

1. **`base_height`**: base_link (the body centre) above the floor while standing. The sector
   scans measure their height band from the floor `base_height` below base_link; 0.48 m is the
   model's leg geometry in a normal stance.
2. **The LiDAR mount**: `rostopic echo -n 1 --noarr /livox/lidar/header` gives the frame the
   X30's driver publishes in; set `lidar_frame` and `lidar_pose` so that a wall 1 m ahead shows at
   1 m in `perception/safety_scan` (`rostopic echo -n 1 /perception/safety_scan`).
3. **Braking**: `limits/max_decel`, `max_yaw_accel` and `reaction_time`, worst case (floor,
   payload, battery).

## Test

```bash
catkin_make run_tests_x30_bringup && catkin_test_results build/test_results/x30_bringup
```

`stack_base.test` runs the stack's base end to end against the simulated X30
(`x30_driver/test/fake_robot.py`): bringup with the driver, the control mux and the sector scans;
Livox scans in, `control/nav_cmd_vel` walks the robot, a wall ahead stops it through the
protective field, the e-stop holds it until a release and a re-arm.

`stack_navigation.test` walks the simulated X30 along a route: bringup with the driver (leg
odometry on TF), the mux, the scans and the navigation, `map` -> `odom` fixed; `navigation/set_path`
and `navigation/start` take the robot through the follower, the mux and the driver to the end of a
1.2 m route.

`test_launch.py` checks the layers behind their arguments (`roslaunch --nodes`, `--dump-params`):
the robot alone by default, every layer when asked, the leg odometry topic following the
namespace, and `leg_odom_tf` with `localization` refused.
