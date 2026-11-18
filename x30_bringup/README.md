# x30_bringup

The Jueying X30 in one launch: [x30_driver](../x30_driver/README.md) and
`robot_state_publisher` with the URDF of [x30_description](../x30_description/README.md) in one
namespace, the robot profile every node of the stack reads, and optionally the control mux
([x30_control](../x30_control/README.md)), the LiDAR sector scans
([x30_perception](../x30_perception/README.md)), FAST-LIO and its monitor
([x30_odometry](../x30_odometry/README.md)), the localization
([x30_localization](../x30_localization/README.md)), the site maps
([x30_mapping](../x30_mapping/README.md)), the navigation
([x30_navigation](../x30_navigation/README.md)), the health monitor
([x30_health](../x30_health/README.md)), the inspection at the stop points
([x30_inspection](../x30_inspection/README.md)) and its cameras. With `sim:=true` the
simulated X30 of [x30_sim](../x30_sim/README.md) takes the driver's place, and the LiDARs', the
IMU's and FAST-LIO's.

## Usage

```bash
roslaunch x30_bringup bringup.launch                                  # namespace x30
roslaunch x30_bringup bringup.launch params_file:=$HOME/x30_driver_params.yaml rviz:=true
roslaunch x30_bringup bringup.launch control:=true perception:=true leg_odom_tf:=true
roslaunch x30_bringup bringup.launch control:=true perception:=true \
    odometry:=true localization:=true mapping:=true site:=gardu-a navigation:=true
# navigation without a map: the odometry start (the dock) is the origin (Odometry only)
roslaunch x30_bringup bringup.launch control:=true perception:=true odometry:=true \
    localization:=true mapping:=true navigation:=true odometry_only:=true site:=gardu-a
# the same stack on the simulated X30, its site drawn from the world
roslaunch x30_bringup bringup.launch sim:=true site:=mall control:=true perception:=true \
    odometry:=true localization:=true mapping:=true navigation:=true health:=true rviz:=true
# a substation patrol with the inspection at its stop points
export HIKMICRO_PASSWORD='<thermal camera password>' HERTZINO_PASSWORD='<acoustic camera password>'
roslaunch x30_bringup bringup.launch control:=true perception:=true odometry:=true \
    localization:=true mapping:=true site:=gardu-a navigation:=true health:=true \
    inspection:=true hikmicro:=true hertzino:=true
```

With `control:=true` the control mux is the only publisher of `/x30/cmd_vel`: navigation
publishes `control/nav_cmd_vel`, an operator `control/remote/<source>/cmd_vel`, and nothing
else may publish the driver's `cmd_vel` any more.

## Launch

`launch/bringup.launch`:

| Argument | Default | |
|---|---|---|
| `namespace` | `x30` | driver topics and services, `robot_description` |
| `sim`, `world`, `scenario` | `false`, empty, empty | `sim_robot_node` (x30_sim) in place of x30_driver, the LiDARs, the IMU and FAST-LIO, in that world and scenario (empty: mall_loop, none); `site` is drawn from the world unless it has a map (never with `odometry_only`); the odometry layer runs with `odometry_method:=external` (`fast_lio` is refused) |
| `params_file` | [`x30_driver/config/driver_params.yaml`](../x30_driver/config/driver_params.yaml) | a copy with this robot's settings, e.g. `invert_wz` |
| `overrides_file` | empty | a few parameters on top, e.g. `x30_driver/config/fake_robot.yaml` |
| `robot_params` | [`config/robot.yaml`](config/robot.yaml) | the robot profile, loaded as `/robot` |
| `control` | `false` | the control mux (x30_control), publishing `/<namespace>/cmd_vel` |
| `perception` | `false` | `lidar_sector_node` (x30_perception) and the LiDAR mount TF |
| `leg_odom_tf` | `false` | the driver publishes `odom` → `base_link` (leg odometry, roll and pitch from the IMU): the scans' leveling, while no other odometry owns that transform; refused together with `localization` |
| `odometry`, `odometry_method` | `false`, `fast_lio` (`external` with `sim`) | FAST-LIO behind its start gate and the odometry monitor (x30_odometry); `external`: another estimator publishes `/Odometry` |
| `localization` | `false` | `map` → `odom` and the bridge of FAST-LIO into `odom` → `base_link` (x30_localization) |
| `mapping`, `site` | `false`, empty | the site maps, mapping sessions and routes (x30_mapping); the site at start |
| `odometry_only` | `false` | navigation without a map, with `localization` ([Odometry only](#odometry-only)): FAST-LIO's start is the map origin, `site` is created without a map, the state stays `LOCALIZED`; with `sim` nothing is drawn from the world and the simulator sends no `/initialpose` |
| `sites_dir` | empty (`/tmp/x30_sim/sites` with `sim`) | the sites of localization, mapping and navigation; empty: their files' (`~/x30/map`) |
| `navigation` | `false` | the planner, route loader, path follower, avoidance and docking (x30_navigation), commanding the control mux; the robot's battery and odometry under `namespace` |
| `health` | `false` | `health_monitor_node` (x30_health), told the nodes, sensors and states of the layers started here; a missing node is an error (speed scale 0), so the inspection and its cameras are not among them |
| `inspection`, `reports_dir` | `false`, empty (`/tmp/x30_sim/reports` with `sim`) | `inspection_node` (x30_inspection) on the `sites_dir`, the path follower's `hooks/inspection` (180 s for a whole point); the reports there, empty: its file's (`~/x30/reports`) |
| `hikmicro`, `hikmicro_host`, `hikmicro_password` | `false`, `192.168.1.64`, empty | the HIKMICRO thermal PTZ camera (hikmicro_driver, from x30.repos); leave the password empty: the driver reads `HIKMICRO_PASSWORD` (a password argument goes to the parameter server, which roslaunch prints) |
| `hertzino`, `hertzino_host`, `hertzino_password` | `false`, `192.168.2.204`, empty | the Hertzino acoustic camera (hertzino_driver, from x30.repos); leave the password empty: the driver reads `HERTZINO_PASSWORD` |
| `lidar_frame`, `lidar_pose` | `livox_frame`, `0 0 0 0 0 0` | `base_link` → the frame of `/livox/lidar` (`x y z yaw pitch roll`, m and rad) |
| `rviz` | `false` | RViz with the robot model |

## Nodes

| Node | Namespace | |
|---|---|---|
| `x30_driver` | `x30` | `cmd_vel` in; `odom`, `imu`, `joint_states`, `battery`, `state`, `diagnostics`; Trigger services |
| `sim_robot_node` (`sim:=true`, instead) | `/` | `/<namespace>/cmd_vel` in; `/<namespace>/odom`, `battery`, `sit_down`, `stand_up`, `charge_*`; `/livox/lidar`, `/imu/data`, `/Odometry`, `/cloud_registered_body`, `sim/*` |
| `robot_state_publisher` | `x30` | TF `base_link` → `TORSO` → legs, `imu_link`, `LIDAR_*` from `joint_states` |
| `control_mux_node` (`control:=true`) | `/` | `control/*` in, `/<namespace>/cmd_vel` out |
| `lidar_sector_node`, `lidar_mount` (`perception:=true`) | `/` | `/livox/lidar` in, `perception/sector_scan`, `perception/safety_scan` out |
| `fast_lio` (in `fast_lio/`), `odometry_monitor_node` (`odometry:=true`) | `/` | `/livox/lidar`, `/imu/data` in; `/Odometry`, `camera_init` → `body`, `odometry/*` out |
| `localization_node` (`localization:=true`) | `/` | `map` → `odom`, `odom` → `camera_init`, `body` → `base_link`; `localization/*` |
| `grid_mapper_node`, `site_manager_node`, `route_recorder_node` (`mapping:=true`) | `/` | `/map`, `mapping/*`, `site/*`, `route/*` |
| `planner_node`, `path_loader_node`, `path_follower_node`, `obstacle_avoidance_node`, `docking_node` (`navigation:=true`) | `/` | `planner/*`, `navigation/*`, `docking/*`; `control/nav_cmd_vel` out |
| `health_monitor_node` (`health:=true`) | `/` | `health/status`, `diagnostics` |
| `inspection_node` (`inspection:=true`) | `/` | `inspection/inspect` (the follower's hook), `inspection/add_target`, `inspection/report`, `inspection/status` |
| the camera drivers (`hikmicro:=true`, `hertzino:=true`) | `hikmicro`, `hertzino` | see their READMEs |
| `rviz` (`rviz:=true`) | `x30` | |

The driver publishes no TF by default (`publish_tf: false`): `odom` → `base_link` belongs to
the odometry or localization in use; `leg_odom_tf:=true` hands it to the driver. With
`localization:=true` the tree is `map` → `odom` → `camera_init` → `body` → `base_link`, and the
launch refuses `leg_odom_tf:=true` (`base_link` would have two parents).

## Odometry only

`odometry_only:=true` walks taught routes without a map, like HotDog and Pashupati. FAST-LIO's
start is the map origin (localization's `assume_start_origin`), the site is created without a
map (the site manager's `create_site`), routes are taught and repeated in that frame, and
nothing is mapped: the grid mapper stays idle. It is a mode of `localization:=true`.

```bash
roslaunch x30_bringup bringup.launch control:=true perception:=true odometry:=true \
    localization:=true mapping:=true navigation:=true health:=true \
    odometry_only:=true site:=gardu-a
rostopic echo -n 1 /localization/status    # LOCALIZED, source origin, once FAST-LIO runs
# teach: walk the route, a stop point wherever the robot inspects
rosservice call /route/start
rosservice call /route/mark_stop_point "{dwell_sec: 10.0}"
rosservice call /route/stop                # <sites_dir>/gardu-a/routes/<date>_<time>.csv
# repeat: on the route's start, in AUTO
rosservice call /navigation/load_path "{path: gardu-a/routes/<date>_<time>.csv}"
rosservice call /navigation/start
```

Walk the route on the X30's handheld (manual mode is a takeover the driver latches: hand back
with `rosservice call /x30/non_manual_mode` before the repeat) or through the control mux
(`control/set_mode` remote, `control/remote/<source>/cmd_vel`, then `control/set_mode` auto).

- **Start on the same spot.** The map frame is where FAST-LIO started: boot the robot, and so
  start FAST-LIO behind its gate, on the same spot every time (the dock), facing the same way.
  One degree off at the start is 1.7 cm per metre on every route (0.5 m at 30 m).
- **Only tags correct the drift.** Without a map the error grows with the distance walked; the
  state stays `LOCALIZED` (`max_sigma_xy` 100 m, `max_sigma_yaw_deg` 180°) and
  `localization/status` reports the drift as sigma. Keep the rounds short, or record tags along
  the routes with a tag detector running (x30_perception's `fiducials` on a calibrated camera):
  `localization/record_tags` with a tag in view, best near the start where the drift is small;
  seen again on later rounds, they correct `map` → `odom`.
- **An odometry restart moves the origin.** FAST-LIO starting over (`odometry/restarted`) drops
  localization to `NONE` and pauses the follower. Walk the robot back to the start spot and give
  that spot as the origin:
  `rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'`
  (a recorded tag in view relocalizes too). `localization/assume_origin` takes FAST-LIO's start
  as the origin, which is right only while FAST-LIO has run since the start spot; a respawned
  `localization_node` does the same.
- **No planner.** `planner/go_to` plans on a map and refuses without one. Routes, stop points
  with their inspection, avoidance, the protective field and docking need none.

A site can still be mapped later (`mapping/start`), in this frame.

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

`stack_sim.test` runs the whole stack on x30_sim: control, perception, the odometry monitor on
the simulated FAST-LIO, localization, mapping on a site drawn from the world, navigation and
health. The drift scenario makes FAST-LIO 1 % long with a yaw bias; the robot localizes from the
simulator's `/initialpose`, walks to a goal around a corner (`planner/go_to`), its camera's
detector sees the tag on the east wall on the way, and it arrives within 0.15 m of the goal by
its true pose (0.25 m without the tag), without touching anything.

`stack_odometry.test` runs the same stack with `odometry_only:=true`: the site is created
without a map and the robot is localized at the odometry start (the map origin, not the world's
frame); the operator teaches a 3 m route on the stick in REMOTE with a stop point at its end,
walks back, and the follower repeats it in AUTO to within 0.15 m of the taught stop by the true
poses, localized all along. Both sim tests keep what the nodes remember across runs (site, last
route, trip) in their run's directory.

`test_launch.py` checks the layers behind their arguments (`roslaunch --nodes`, `--dump-params`):
the robot alone by default, every layer when asked, the leg odometry topic following the
namespace, `leg_odom_tf` with `localization` refused, health expecting exactly the nodes
started, the inspection as the follower's hook and not expected by health, `sim` in place of
the driver and FAST-LIO with `sites_dir` and `reports_dir` reaching every layer, and
`odometry_only`'s parameters (refused without `localization`).

`test_tools.py` checks the robot tools of [tools/](../tools/README.md): Deep Robotics' autonomy
off and on against a fake jy_cog, the offline dependency check and its package lists, and the
source archive.
