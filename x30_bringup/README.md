# x30_bringup

The Jueying X30 in one launch: [x30_driver](../x30_driver/README.md) and
`robot_state_publisher` with the URDF of [x30_description](../x30_description/README.md) in one
namespace, the robot profile every node of the stack reads, and optionally the control mux
([x30_control](../x30_control/README.md)), the LiDAR sector scans
([x30_perception](../x30_perception/README.md)), FAST-LIO and its monitor
([x30_odometry](../x30_odometry/README.md)), the localization
([x30_localization](../x30_localization/README.md)), the sites, their routes and the
navigation ([x30_navigation](../x30_navigation/README.md)), the health monitor
([x30_health](../x30_health/README.md)), the inspection at the stop points
([x30_inspection](../x30_inspection/README.md)) and its cameras. With `sim:=true` the
simulated X30 of [x30_sim](../x30_sim/README.md) takes the driver's place, and the LiDARs', the
IMU's and FAST-LIO's.

## Usage

```bash
roslaunch x30_bringup bringup.launch                                  # namespace x30
roslaunch x30_bringup bringup.launch params_file:=$HOME/x30_driver_params.yaml rviz:=true
roslaunch x30_bringup bringup.launch control:=true perception:=true leg_odom_tf:=true
# navigation without a map: the odometry start (the dock) is the origin (No map)
roslaunch x30_bringup bringup.launch control:=true perception:=true \
    odometry:=true localization:=true site:=gardu-a navigation:=true
# the same stack on the simulated X30, its site (tags, patrol loop) drawn from the world
roslaunch x30_bringup bringup.launch sim:=true site:=mall control:=true perception:=true \
    odometry:=true localization:=true navigation:=true health:=true rviz:=true
# a substation patrol with the inspection at its stop points
export HIKMICRO_PASSWORD='<thermal camera password>' HERTZINO_PASSWORD='<acoustic camera password>'
roslaunch x30_bringup bringup.launch control:=true perception:=true odometry:=true \
    localization:=true site:=gardu-a navigation:=true health:=true \
    inspection:=true hikmicro:=true hertzino:=true
```

With `control:=true` the control mux is the only publisher of `/x30/cmd_vel`: navigation
publishes `control/nav_cmd_vel`, an operator `control/remote/<source>/cmd_vel`, and nothing
else may publish the driver's `cmd_vel` any more.

## Two hosts

The X30 Pro has two computers for software (Application Manual chapter 1): the **navigation
host** (192.168.1.106, the intelligent controller on the back, x86 with an NVIDIA GPU) and the
**perception host** (192.168.1.105, inside the body, an RK3588). Both are on the robot's switch
(192.168.1.x) and on the LiDAR switch (192.168.2.x), and both run Deep Robotics' LiDAR and IMU
drivers from power-up, each into its own ROS master (API documentation 2.1 and 3.1:
`/livox/lidar`, `/lidar_points`, `/imu/data`). The stack uses the navigation host's: FAST-LIO and
the protective field's scans on the computer their LiDAR data arrives on, no network between
them. It never runs a LiDAR driver of its own (a second driver would fight Deep Robotics' for
the LiDAR), and the perception host needs none for it: its own stays for Deep Robotics'
elevation map and charging. The stack is split by what each part needs to be next to:

| | navigation host (.106): `navigation_host.launch` | perception host (.105): `perception_host.launch` |
|---|---|---|
| runs | the driver, `robot_state_publisher`, the control mux, the LiDAR scans, FAST-LIO, localization, the site and its routes ([No map](#no-map)), navigation (NeuPAN with `controller:=neupan`), health, the inspection with its sites and reports, the handheld's UDP gateway (x30_bridge, `bridge:=false` off) | the inspection cameras: hikmicro_driver (PTZ, RTSP video), hertzino_driver; their recorder (x30_recorder: raw thermal frames, video, acoustic data, `recorder:=false` off) |
| why | the motion host reports to its UDP 43897; the protective field stays on the same computer as the LiDAR driver and the mux, so the network between the hosts never blinds it; FAST-LIO and NeuPAN want its x86 cores | decoding the cameras' video off the computer that drives; the inspection calls the cameras' services, small messages; the recordings written where the cameras' data arrives, never over the network |
| ROS | the robot's master (Deep Robotics' roscore, kept by `dr_services.sh`), `ROS_IP=192.168.1.106` | the navigation host's master, `ROS_IP=192.168.1.105`; Deep Robotics' own master here stays for its own nodes |
| Deep Robotics | `tools/dr_services.sh off`: its autonomy, URDF and bridge off | untouched: the elevation map, charging and its drivers |
| build (`x30_host.sh build`) | the whole workspace | x30_utils, x30_msgs, x30_recorder, x30_bringup and the camera drivers only |

`tools/x30_host.sh` builds each host's workspace (`build`), sets its ROS network (`env`),
checks that the two see each other (`check`: the master, the other host, every registered
node's address, how far the two clocks are apart; Deep Robotics' autonomy on the navigation
host, the cameras' ports on the perception host), and runs each host's launch file by hand or as
a systemd service under a watchdog (`start`, `install`). **The role is the host's own, by its
address**: Deep Robotics names the navigation host `ysc-perception`, so the names mislead; on the
robot the other host's role is refused, and `install` removes the other host's service where an
older x30_host.sh let it in. On each host, `~/x30_ws` holding x30_ros and x30.repos in `src`:

```bash
cd ~/x30_ws/src/x30_ros/tools
bash x30_host.sh build
eval "$(bash x30_host.sh env)"
bash x30_host.sh check
```

By hand, the navigation host's (.106) first, then the perception host's (.105) after its `env`:

```bash
roslaunch x30_bringup navigation_host.launch site:=gardu controller:=neupan
roslaunch --wait x30_bringup perception_host.launch
```

Or as a service, started at boot, its settings in `/etc/x30/<role>.env`; `install` prints the
next lines (the settings to edit, then `sudo systemctl restart x30-<role>` and its journal):

```bash
sudo bash x30_host.sh install
```

`build` is catkin_make as the robot needs it: ROS sourced, `PYTHONNOUSERSITE=1` (a setuptools
that `pip install --user` put in `~/.local` stops catkin), Release, no tests, at most one job per
2 GB of memory, on the perception host its packages only, and a workspace `CMakeLists.txt` that
is not catkin's (left by an older workspace) replaced, kept as `CMakeLists.txt.orig`; its
arguments go to catkin_make (`--pkg x30_navigation`).

**The watchdog** (`tools/x30_watchdog.py`, the service's main process): it starts the launch
file, tells systemd ready once the master answers and the critical nodes do (on the navigation
host: the driver, the control mux, the LiDAR scans and the path follower), then checks them every
5 s and pings systemd's watchdog (`WatchdogSec=60`). The service restarts (`Restart=on-failure`)
when the master restarted (its nodes are lost with it: the perception host follows the
navigation host's restarts this way), when the master or a critical node stays away longer than
45 s (a respawn takes seconds), when the start takes over 4 min, when roslaunch exits, or when
the watchdog itself hangs. Meanwhile the robot stands: the control mux stops a command that is
not renewed, and the motion host stops taking commands without the driver's heartbeat.

**Deep Robotics' boot**: the navigation host's service starts after Deep Robotics' services
(`install` orders it after every `jy_*` service and the one running the ROS master, as it finds
them), then runs `dr_services.sh off` until their boot has settled (a round 30 s after the first
finds nothing to stop) and waits for their master. While the stack runs the watchdog sweeps
every 30 s (`dr_services.sh sweep`): what one of their services starts again later is stopped
again and named in the journal, without Deep Robotics' own kill scripts, which may stop
processes by name.

**The clocks**: `check`'s `clock` line compares the other host's stamps with this host's clock.
No node of x30_ros compares one host's stamps with the other's clock, so a skew stops nothing;
logs, reports and bags of the two hosts disagree by it (tools/README.md, "Two hosts and the
watchdog", on keeping one clock).

Nodes started by others without `ROS_IP` (Deep Robotics' drivers on the navigation host
register as `ysc-perception`) are reachable from the other host only if that name resolves
there: `check` lists them, and a line `192.168.1.106 ysc-perception` in the perception host's
`/etc/hosts` fixes it.

## Running for months

What a robot on patrol around the clock needs beside the stack, all on by default:

| | where | what it keeps |
|---|---|---|
| restarts | the service (`StartLimitIntervalSec=0`), the watchdog, every node's `respawn` | the stack never given up on |
| Deep Robotics' autonomy | the watchdog's sweep every 30 s | off, whatever of it starts late |
| inspection reports | x30_inspection, after every report | whole days, the oldest first, while less than 6 GB is free (`reports_min_free_gb`, `reports_keep_days`) |
| camera recordings | x30_recorder on the perception host, before a session and every 30 s of it | the oldest sessions first while less than 10 GB is free (`disk/min_free_gb`, `disk/keep_days`); nothing recorded below 3 GB |
| ROS's logs (`~/.ros/log`) | the service, at every start (`tools/x30_logs.py`) | within 2 GB, the oldest runs first (`X30_ROS_LOG_MB`) |
| each node's log | `X30_LOG_DIR` (`~/x30/logs`) | one JSON lines file per node, rotated at 10 MB, 5 kept |
| black box | `navigation_host.launch blackbox:=true` (`scripts/blackbox`) | the last 3 h of every layer's status, the commands before and after the mux, the pose, the battery, the safety scan, the inspection and `/rosout_agg`, in 10 min rosbag splits in `~/x30/blackbox` (about 0.5 GB) |
| the disk, the CPU, the temperatures | x30_health | warnings below 5 GB free and above the limits |

After an incident: `rqt_bag ~/x30/blackbox/x30_<time>.bag` for what the robot was told and
did, `~/x30/logs/<node>.jsonl` for why. Copy the reports off the robot before they are old
enough to go.

## Launch

`launch/bringup.launch`:

| Argument | Default | |
|---|---|---|
| `namespace` | `x30` | driver topics and services, `robot_description` |
| `sim`, `world`, `scenario` | `false`, empty, empty | `sim_robot_node` (x30_sim) in place of x30_driver, the LiDARs, the IMU and FAST-LIO, in that world and scenario (empty: mall_loop, none); `site` gets the world's tags and route, in the frame of the robot's start, unless it has its tags already; the odometry layer runs with `odometry_method:=external` (`fast_lio` is refused) |
| `params_file` | [`x30_driver/config/driver_params.yaml`](../x30_driver/config/driver_params.yaml) | a copy with this robot's settings, e.g. `invert_wz` |
| `overrides_file` | empty | a few parameters on top, e.g. `x30_driver/config/fake_robot.yaml` |
| `robot_params` | [`config/robot.yaml`](config/robot.yaml) | the robot profile, loaded as `/robot` |
| `control` | `false` | the control mux (x30_control), publishing `/<namespace>/cmd_vel` |
| `perception` | `false` | `lidar_sector_node` (x30_perception) and the LiDAR mount TF |
| `leg_odom_tf` | `false` | the driver publishes `odom` → `base_link` (leg odometry, roll and pitch from the IMU): the scans' leveling, while no other odometry owns that transform; refused together with `localization` |
| `odometry`, `odometry_method` | `false`, `fast_lio` (`external` with `sim`) | FAST-LIO behind its start gate and the odometry monitor (x30_odometry); `external`: another estimator publishes `/Odometry` |
| `localization` | `false` | `map` → `odom` and the bridge of FAST-LIO into `odom` → `base_link` (x30_localization): FAST-LIO's start is the map origin ([No map](#no-map)) |
| `sites_dir` | empty (`/tmp/x30_sim/sites` with `sim`) | the sites of localization, the site manager and navigation; empty: their files' (`~/x30/map`) |
| `navigation`, `site` | `false`, empty | the sites (routes and tags, no map) with route teaching, the route loader, path follower, avoidance and docking (x30_navigation), commanding the control mux; the robot's battery and odometry under `namespace`; the site at start, created when it is missing |
| `health` | `false` | `health_monitor_node` (x30_health), told the nodes, sensors and states of the layers started here; a missing node is an error (speed scale 0), so the inspection and its cameras are not among them |
| `inspection`, `reports_dir` | `false`, empty (`/tmp/x30_sim/reports` with `sim`) | `inspection_node` (x30_inspection) on the `sites_dir`, the path follower's `hooks/inspection` (180 s for a whole point); the reports there, empty: its file's (`~/x30/reports`) |
| `hikmicro`, `hikmicro_host`, `hikmicro_password` | `false`, `192.168.1.64`, empty | the HIKMICRO thermal PTZ camera (hikmicro_driver, from x30.repos); leave the password empty: the driver reads `HIKMICRO_PASSWORD` (a password argument goes to the parameter server, which roslaunch prints) |
| `hertzino`, `hertzino_host`, `hertzino_password` | `false`, `192.168.2.204`, empty | the Hertzino acoustic camera (hertzino_driver, from x30.repos); leave the password empty: the driver reads `HERTZINO_PASSWORD` |
| `lidar_frame`, `lidar_pose` | `livox_frame`, `0 0 0 0 0 0` | `base_link` → the frame of `/livox/lidar` (`x y z yaw pitch roll`, m and rad) |
| `bridge` | `false` (`navigation_host.launch`: `true`) | the handheld's UDP gateway (x30_bridge, UDP 9870, the control mux's `handheld` source); its shared secret is `X30_TELEOP_TOKEN` in the environment, never an argument |
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
| `site_manager_node`, `route_recorder_node`, `path_loader_node`, `path_follower_node`, `obstacle_avoidance_node`, `docking_node` (`navigation:=true`) | `/` | `site/*`, `route/*`, `navigation/*`, `docking/*`; `control/nav_cmd_vel` out |
| `health_monitor_node` (`health:=true`) | `/` | `health/status`, `diagnostics` |
| `inspection_node` (`inspection:=true`) | `/` | `inspection/inspect` (the follower's hook), `inspection/add_target`, `inspection/report`, `inspection/status` |
| the camera drivers (`hikmicro:=true`, `hertzino:=true`) | `hikmicro`, `hertzino` | see their READMEs |
| `teleop_udp_node` (`bridge:=true`) | `/` | UDP 9870 for the handheld; `control/remote/handheld/cmd_vel` out, the stack's services and status in |
| `rviz` (`rviz:=true`) | `x30` | |

The driver publishes no TF by default (`publish_tf: false`): `odom` → `base_link` belongs to
the odometry or localization in use; `leg_odom_tf:=true` hands it to the driver. With
`localization:=true` the tree is `map` → `odom` → `camera_init` → `body` → `base_link`, and the
launch refuses `leg_odom_tf:=true` (`base_link` would have two parents).

## No map

The X30 walks taught routes without a map, like HotDog and Pashupati: nothing in the stack
maps, there is no `/map` and no planner. FAST-LIO's start is the map origin (localization's
`assume_start_origin`, its default), the launch `site` is created when it is missing (the site
manager's `create_site`, its default), and routes are taught and repeated in that frame. A site
is a folder of routes, tags and inspection points (x30_navigation); an old `odometry_only:=` or
`mapping:=` of another version in a service's settings is ignored.

```bash
roslaunch x30_bringup bringup.launch control:=true perception:=true odometry:=true \
    localization:=true navigation:=true health:=true site:=gardu-a
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
  One degree off at the start is 1.7 cm per metre on every route (0.5 m at 30 m). The origin
  is as sure as that spot (`estimator/origin_sigma_xy` 5 cm, `origin_sigma_yaw_deg` 2°): a
  recorded tag seen soon after the start corrects it.
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
- **No planner.** Routes, stop points with their inspection, path shift and the detours around
  obstacles, the protective field and docking need no map.

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
the simulated FAST-LIO, localization from the robot's start, the site drawn from the world (its
tags and patrol loop, in the frame of the start, no map), navigation and health. The drift
scenario makes FAST-LIO 1 % long with a yaw bias; the robot walks the 30 m loop, its camera's
detector sees the tag on the east wall on the way, and it ends the loop within 0.15 m by its
true pose, localized within 0.10 m, without touching anything.

`stack_odometry.test` runs the same stack for teach and repeat: the robot is localized at the
odometry start; the operator teaches a 3 m route on the stick in REMOTE with a stop point at
its end, walks back, and the follower repeats it in AUTO to within 0.15 m of the taught stop by
the true poses, localized all along. The sim tests keep what the nodes remember across runs
(site, last route, trip) in their run's directory.

`test_launch.py` checks the layers behind their arguments (`roslaunch --nodes`, `--dump-params`):
the robot alone by default, every layer when asked, the leg odometry topic following the
namespace, `leg_odom_tf` with `localization` refused, health expecting exactly the nodes
started, the inspection as the follower's hook and not expected by health, `sim` in place of
the driver and FAST-LIO with `sites_dir` and `reports_dir` reaching every layer, no map (the
odometry start the origin, the site created, no mapper nor planner), the controller reaching
the path follower, and each host's launch file: the navigation host all but the cameras (and
every node its watchdog watches), the perception host the cameras at their addresses (stand-in
packages for the drivers).

`stack_neupan.test` (registered when `X30_PYTHON` points at NeuPAN's venv) walks the patrol loop
of x30_sim with `controller:=neupan`, once the scenario's person has crossed the south
corridor (it walks into whatever stands in its way): the box it leaves on the route the robot
walks around (path shift picks the side, NeuPAN keeps the clearance) back to the loop's end
within 0.3 m, without touching anything.

`test_tools.py` checks the robot tools of [tools/](../tools/README.md): Deep Robotics' autonomy
off and on against a fake jy_cog, the offline dependency check (the perception host's too) and
its package lists, and the source archive. `test_hosts.py`: `x30_host.sh`'s network, service
file and start, and the watchdog against a fake master and node: ready once the nodes answer,
a restart when the master restarts, when a critical node stays away longer than the grace or
the start never comes up, and the launch's exit passed on.
