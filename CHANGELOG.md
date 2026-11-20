# Changelog

All notable changes to x30_ros. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased] (0.14.0)

### Added
- A black box on the navigation host (`navigation_host.launch blackbox:=true`, the default;
  `scripts/blackbox`): the last 3 h of every layer's status, the commands before and after the
  control mux, the pose, the battery, the safety scan, the inspection and `/rosout_agg` in a
  rolling rosbag, 10 min splits in `~/x30/blackbox` (`blackbox_dir`, `blackbox_minutes`,
  `blackbox_splits`).
- Report retention (x30_inspection): at start and after every report whole days go, never
  today's: older than `reports_keep_days` (0: kept), then the oldest while less than
  `reports_min_free_gb` (6 GB) is free.
- The service keeps ROS's logs within `X30_ROS_LOG_MB` (2 GB, `tools/x30_logs.py`: the oldest
  runs first, the newest 5 kept) and writes each node's JSON lines log to `X30_LOG_DIR`
  (`~/x30/logs`, rotated) by default.
- The follower's check of the control field compares the turning room too (`field/yaw_margin`).
- A robot without a map: mapping.launch `grid:=false` (the site manager and the route recorder
  alone, the site manager's `mapping: false` offering no `mapping/*`), navigation.launch
  `planner:=false`.

### Changed
- The navigation host never maps: `navigation_host.launch` is odometry only, with no
  `odometry_only` or `mapping` argument (old ones in a service's settings are ignored); it runs
  the site and its routes, no grid mapper, no mapping session, no planner. `odometry_only:=true`
  in bringup.launch maps nothing either, and x30_health expects only what runs.
- The arc check of a turning translation moved from x30_control to `x30_utils.safety`: the
  field and the follower's guard share it.
- The follower's `safety/yaw_margin_deg` is 6 (was 5): at least the field's 0.10 rad.

### Fixed
- The follower's guard was less cautious than the control mux's field, so the mux cut what the
  follower sent. The guard now checks the turning arc as the field does (a turn that would sweep
  into a point goes on straight and turns later) and checks what is published: the shaper's
  ramps never outrun it, a ramp bent toward something goes the guarded way. NeuPAN past a box:
  35 of 500 commands limited by the field, now none; the pocket of the detour tests, driven
  through the field: stopped 397 times and stuck in the gap, now through without a cut.
- systemd stopped restarting a service that failed 5 times in 10 s: `StartLimitIntervalSec=0`.
- Log floods: localization logged each correction of a tag in view (twice a second), now once a
  minute per tag; the control mux logged each change of the protective field (dozens a second
  beside a person), now each state at most once a second.

## [Unreleased] (0.13.0)

### Added
- Two hosts (x30_bringup): `navigation_host.launch` for the navigation host (192.168.1.106: the
  driver, control, the LiDAR scans, FAST-LIO, localization, mapping, navigation, health, the
  inspection) and `perception_host.launch` for the perception host (192.168.1.105: the
  inspection cameras), on the navigation host's ROS master.
- `tools/x30_host.sh env|check|start|install`: each host's ROS network, a check that the two see
  each other (master, ping, every node's address, clock), each host's launch file by hand or as
  the systemd service `x30-<role>` with its settings in `/etc/x30/<role>.env`; on the navigation
  host Deep Robotics' autonomy goes off until its boot settled.
- `tools/x30_watchdog.py`: the service's main process, under systemd's watchdog; it restarts the
  host's stack when the master restarted, the master or a critical node (driver, control mux,
  LiDAR scans, path follower) stays away past a grace, the start never comes up or roslaunch
  exits.
- `tools/check_deps.sh --perception`: the perception host's camera drivers only, with a build
  line for them; on the navigation host a row for NeuPAN's venv.
- `tools/dr_services.sh sweep`: what of Deep Robotics' autonomy runs again, stopped, one line
  each, without its kill scripts. The navigation host's watchdog runs it every 30 s
  (`x30_watchdog.py --sweep`), so what one of Deep Robotics' services starts again late goes
  again and is named in the journal.
- `x30_host.sh install navigation` orders the service after Deep Robotics' services (`jy_*`)
  and the one running the ROS master, as found on the robot.
- `x30_host.sh check` (its ROS rows in `tools/x30_check.py`): how far the other host's clock is
  from this one's (the stamps of a small topic it publishes on arrival), the systemd service
  running the master and Deep Robotics' autonomy on the navigation host, the cameras' ports on
  the perception host (at the addresses of the service's launch arguments).

## [Unreleased] (0.12.0)

### Added
- Path follower controller `neupan` (x30_navigation): [NeuPAN](https://github.com/hanruihua/NeuPAN)
  tracks the route ahead and keeps the footprint off the safety scan's points in one MPC,
  planning forward and sideways speed and the yaw rate (the `holonomic` kinematics patched into
  NeuPAN for the X30, `tools/neupan`; `diff` for a robot that cannot strafe), within the robot
  profile's speeds and accelerations; a solve every 0.1 s. Reactive avoidance stands by for it,
  path shift still picks the side; NeuPAN or its patches missing: the follower says how to
  install them. `controller:=` for navigation.launch and bringup.launch, `python:=` (default
  `$X30_PYTHON`) for the follower's interpreter.
- DUNE for the X30's footprint (`models/dune_x30.pth`, 1.5 mm mean distance error, at most 8 mm
  too far) with its record (`dune_x30.json`: footprint, training, errors); the controller
  refuses a model of another footprint or points beyond its training range. `train_dune`
  trains one for any robot profile (`--measure` re-measures one).
- `tools/install_neupan.sh`: NeuPAN in a venv beside ROS (PyTorch for the CPU, cvxpy,
  cvxpylayers, NeuPAN's py38 branch with the X30 patches), online or from wheels `--download`
  fetched elsewhere; `--check`.
- Path controllers get the footprint and the accelerations of the robot profile
  (`ControllerLimits`); a controller that keeps its own clearance (`avoids`) turns reactive
  avoidance off.
- `stack_neupan.test` (registered when `X30_PYTHON` is set) and CI's `neupan` job: NeuPAN
  installed as on the robot, its unit tests and the patrol loop around a box on x30_sim.
- NeuPAN's latest solve time in `navigation/status` (`| neupan 23 ms |`); a warning from the
  follower (every 30 s) while more than one solve in ten of the last 50 overruns the step.
- `bench_neupan` (x30_navigation): NeuPAN's solves timed on this computer, the follower's
  controller from the robot profile and navigation.yaml on an open route and in a corridor past
  a box, the robot standing still; exit 1 when the slow ones overrun the step. CI's `neupan` job
  runs it on the runner.

### Fixed
- `X30Node`: unknown parameters are looked for once the node is built, not 0.5 s after it
  started: a slow constructor (the path follower with NeuPAN) no longer gets its own, still
  being declared, reported as typos.

## [Unreleased] (0.11.0)

### Added
- `odometry_only:=true` (x30_bringup): navigation without a map, like HotDog and Pashupati.
  FAST-LIO's start is the map origin (localization's `assume_start_origin`, the state kept
  `LOCALIZED` while only tags correct the drift), the launch site is created without a map
  (x30_mapping's `create_site`, also a mapping.launch argument, exclusive with
  `start_mapping`), routes are taught and repeated in that frame; with `sim` nothing is drawn
  from the world and the simulator sends no `/initialpose`. Refused without `localization`.
- `stack_odometry.test`: on x30_sim, a route taught on the stick without a map and repeated by
  the follower to within 0.15 m of its stop point.

### Fixed
- `planner_node`: the empty `/map` of a site without a map is no `map not usable` error.
- `route_recorder_node`: no `no fresh map->base TF` warning at the first `route/start` while its
  TF listener fills.
- The sim stack tests keep what the nodes remember (site, last route, trip) in their run's
  directory, not in `~/.x30`: parallel and repeated runs no longer share it.

## [Unreleased] (0.10.0)

### Added
- `tools/dr_services.sh status|off|on`: Deep Robotics' own localization, navigation (with its
  local planner), mapping, URDF publisher and bridge (`transfer`: UDP 43897 and `/cmd_vel` to
  the robot) on the navigation host off, now (its `kill_nav.sh` / `kill_mapping.sh`, then what
  is left, all at once) and at boot (`autorun_slam` false, the original kept), and back on; a
  process judged by its program in jy_cog, never its arguments, and what a wrapper script
  started goes with it unless it also runs a kept process; the sensor drivers, the ROS master
  and the robot server kept, the motion and perception hosts never touched. `status` also
  shows who holds UDP 43897 (the driver's reports) and the files that start jy_cog at boot.
- `tools/check_deps.sh`: every apt package, x30.repos source package and the Livox SDK the stack
  needs, checked offline on the robot, with one `apt install` line for what is missing; a
  setuptools in `~/.local` that stops catkin puts `PYTHONNOUSERSITE=1` in its build line.
- `tools/pack_sources.sh`: x30_ros, x30.repos and the Livox SDK in one archive (about 5 MB) for
  a robot without internet; `tools/README.md` on bringing in the packages (`ssh -R` proxy,
  `--print-uris`) and on the navigation host's IMU.
- CI runs shellcheck on `tools/`.

## [Unreleased] (0.9.0)

### Added
- `x30_inspection`: `inspection_node`, substation inspection at the stop points (HotDog's
  pattern). `inspection/inspect`, the path follower's hook, finds the site's inspection point
  where the robot stands (`<site>/inspection.yaml`); per target the HIKMICRO PTZ camera moves
  there through hikmicro_driver's `ptz/move_to` and is waited for until it has arrived, its
  regions (strips, boxes in fractions of the frame, or the frame) are measured from the
  camera's thermometry and judged by the absolute, ambient and similar criteria (NETA MTS), the
  thermal and visible JPEGs kept; partial discharge from one `detect` goal of hertzino_driver,
  with its sound map and PRPD photos. A report folder with `report.json` and the day's
  `findings.csv`, `inspection/report`, `inspection/status` (latched); `inspection/add_target`
  teaches a target from the camera's view.
- `x30_msgs`: `InspectionFinding`, `InspectionReport`, `InspectionStatus`, `AddTarget`.
- `x30_bringup`: `inspection` (and `reports_dir`) starts it on the `sites_dir` as the path
  follower's `hooks/inspection`, 180 s for a whole point; health does not expect it. The camera
  passwords are documented from `HIKMICRO_PASSWORD` and `HERTZINO_PASSWORD`, never an argument.
- CI builds the message packages of hikmicro_driver and hertzino_driver.

## [Unreleased] (0.8.0)

### Added
- `x30_sim`: the X30 in software for the whole stack (the port of dharma_sim). A kinematic body
  behind x30_driver's topics and services (`cmd_vel`, `odom`, `battery`, `sit_down`,
  `stand_up`, `charge_*`); the four Mid-360 of the application manual, tilted as mounted,
  ray-cast into `/livox/lidar` as `livox_ros_driver2/CustomMsg` bytes; the body IMU's specific
  force on `/imu/data`; FAST-LIO per scan (`/Odometry`, `camera_init -> body`,
  `/cloud_registered_body`) and the leg odometry, drifting; a tag camera with its optical frame;
  seeded scenarios (drift, people, dropouts, restarts), the mall world and `draw_site`.
- `x30_bringup`: `sim:=true` (with `world`, `scenario`) runs the stack on x30_sim, the odometry
  layer on its FAST-LIO; `sites_dir` for localization, mapping and navigation;
  `stack_sim.test` walks the whole stack to a goal in the simulator, corrected by a tag.

### Fixed
- `x30_localization`: a tag detection the odometry has not reached yet (FAST-LIO's TF lags its
  scans, the camera does not) waits for it at the next poll instead of being replaced by a newer
  one, ahead again: with a lagging odometry, tags were almost never fused (found in x30_sim).

## [Unreleased] (0.7.0)

### Added
- `x30_health`: `health_monitor_node`, one health signal (`health/status`, latched, with a
  speed scale for the control mux) and diagnostics: sensor rates and header ages read from the
  raw ROS 1 bytes of any type (the header found in the connection's message definition), the
  expected nodes from the master, the stack's typed states, the safety scan, the TF edges and
  the odom chain, the clock, the computer.
- `x30_bringup`: `health` (told the nodes, sensors and states of the layers it starts), and
  `hikmicro` / `hertzino` with their hosts and passwords for the inspection cameras.
- `x30.repos`: FAST-LIO (ROS 1), the Livox message package it is built with, hikmicro_driver
  and hertzino_driver (`vcs import --recursive`).
- `x30_utils`: `create_any_subscription()`, raw bytes of any type with rospy's connection
  header.

### Fixed
- Tests: the private master is taken only on a port no other process answers on (a port found
  free can be bound by a parallel test run before rosmaster binds it), and one master serves the
  process whichever conftest asks for it; status topics are waited for after a service answers
  instead of read at once.

## [Unreleased] (0.6.0)

### Added
- `x30_navigation`: `planner_node` (A* on the site map with a footprint costmap, replanning
  around blockages, a cancel that never waits for a plan), `path_loader_node` (routes from the
  active site), `path_follower_node` (holonomic, pure pursuit, Stanley, LQR, MPPI and PID
  controllers; path shifting, backtracking, stop points with inspection hooks; pause reasons),
  `obstacle_avoidance_node` (VFH, Braitenberg, potential field, follow the gap) and
  `docking_node` (tag approach and the robot's hooks); every command goes to the control mux.
- `x30_bringup`: `navigation` argument; `stack_navigation.test` walks the simulated X30 along a
  route through the follower, the mux and the driver.
- `x30_utils`: every node writes the values it runs with back to the parameter server
  (`rosparam get` shows them, and other nodes can read them); `ServiceClient` (readiness, a
  timeout per call) and `AsyncCall` (answers come back as the node's callbacks); `guard()`;
  `param_notes`, the newest notes of the parameter polls.

### Changed
- `x30_mapping`: service calls through x30_utils' `ServiceClient`.

### Fixed
- `x30_utils`: a read-only value refused on the server came in with the next live change of
  its section; it now stays at its running value until the restart. A change is noted once even
  when a test polls beside the node's own poll.

## [Unreleased] (0.5.0)

### Added
- `x30_odometry`: FAST-LIO2 (ROS 1) on the X30's merged `/livox/lidar` and the body IMU
  `/imu/data` (`config/fast_lio.yaml`: the IMU mount of the Application Manual, the extrinsic
  fixed), its parameters kept in `fast_lio/`; `wait_still`, the launch prefix that holds its
  start and every respawn until the IMU is still and level; `odometry_monitor_node` (INITIALIZING,
  OK, DEGRADED, LOST; restarts, jumps, z drift, a tilted start, the speed test against the leg
  odometry) and the trip; `method:=external` for a simulator, an unknown method fails the launch.
- `x30_localization`: `localization_node`, `map -> odom` from an SE(2) EKF corrected by
  fiducials, pose estimates and scan matching through a chi-square gate, and the gravity-aligned
  bridge of FAST-LIO into `odom -> base_link`; `record_tags` runs beside the timers so
  `map -> odom` keeps flowing while it records.
- `x30_mapping`: `grid_mapper_node`, `site_manager_node`, `route_recorder_node`: sites in
  `~/x30/map` (map, fiducials, routes), mapping sessions with x30_localization, route teaching
  with stop points; service calls between them with a timeout.
- `x30_bringup`: `odometry`, `odometry_method`, `localization`, `mapping` and `site` arguments;
  `leg_odom_tf` together with `localization` is refused (two parents for `base_link`).
- `x30_utils`: `create_service(..., concurrent=True)` for a service that waits for the node's
  other callbacks, and `destroy_subscription()`.

### Fixed
- `x30_utils`: rospy latches one `/tf_static` message per process, so a second node in the
  process replaced the static transforms of the first for late listeners; every node's set now
  stays in the latched message.
- `x30_description`: `imu_link` is the body IMU at (130.45, −1, 14.9) mm from the body centre
  (Application Manual V2.0.4, 6.2); it was at `base_link`.

## [Unreleased] (0.4.0)

### Added
- The X30 stack, the ROS Noetic port of Dharma, in this repository: every package is Dharma's
  algorithms (unchanged, tested without ROS) on a rospy layer.
- `x30_utils`: math, typed configuration, the robot profile, the braking model, file IO, grids,
  logging, and `X30Node`, the rospy base of every node: parameters validated at start (a bad
  value stops the node, exit code 2), live fields polled from the parameter server and
  read-only ones refused with a warning, the robot profile searched up from the node and
  read-only, callbacks one at a time, steady timers, raw subscriptions with a type check, TF,
  and a shutdown that runs the hooks while the topics are open and waits until their messages
  are written. `x30_add_pytests` runs pytest under `catkin_make run_tests`; ROS tests use a
  private `rosmaster`.
- `x30_msgs`: the stack's messages and services (Dharma's interfaces; `TripStatus.since` is a
  ROS 1 `time`).
- `x30_control`: the control mux, the only publisher of `/x30/cmd_vel`: AUTO or REMOTE, speed
  envelope, protective field on the 360° safety scan (translation, rotation, combined arc,
  stale and blind scans), latched e-stop with a hardware input and re-arm, fail-safe on any
  callback error, zeros on shutdown.
- `x30_perception`: `lidar_sector_node` reads the X30's `/livox/lidar` as ROS 1 CustomMsg bytes
  (no Livox package needed, the publisher's type checked) or any PointCloud2, and publishes the
  front and the fail-safe 360° sector scans; optional depth camera; `aruco_detector_node` with
  a `topic_tools` throttle.
- `x30_bringup`: the X30's robot profile (`config/robot.yaml`, loaded as `/robot`), and
  `control`, `perception`, `leg_odom_tf`, `lidar_frame` / `lidar_pose` arguments;
  `stack_base.test` drives the simulated X30 through the mux and the scans.
- `ruff.toml`: lint and format of the stack packages (Python 3.8 target).

## [0.3.1] - 2026-10-18

### Fixed (safety)
- `x30_driver`: only a quiet `cmd_vel` ends an episode. Before, a hold, a takeover or an
  emergency stop or fall ended it, so the stream that kept running started a new one after the
  gate opened and one zero: after a handback, a recovered fall or `stop_stepping` then
  `stand_up`, the driver toggled stepping on its own and walked. An episode now starts with the
  first motion after quiet whatever the gate shows; its switches run only if the gate is open
  then.
- `x30_driver`: at least one zero goes out after motion. The zero burst started at the period
  that sent the last motion and needed a fresh stepping report, so a late period, a
  `stop_burst_sec` shorter than one period or a stale motion state report lost every zero and
  the robot walked on until its own 1 s axis timeout. The burst now starts at the first period
  without motion and sends zeros while the robot steps or its state is unknown (axis and topic
  mode).
- `x30_driver`: `sit_down` on a robot that already sits holds the velocity stream too (before,
  it answered "already sitting" without the hold).
- `x30_driver`: the episode's mode and stepping tries have deadlines from the episode start
  (2 s; 4 × `transition_timeout_sec`). Before, a report that was stale for a while got its
  remaining try whenever it came back, however late in the episode.
- `x30_driver`: a hold that comes in while `stand_up` or `start_stepping` runs (an
  `emergency_stop` on its own service thread) is no longer released by them; their answer says
  the hold was kept.

## [0.3.0] - 2026-10-18

One rule set for the Deep Robotics drivers (x30_ros, lite3_ros2, m20_ros2).

### Fixed (safety)
- `x30_driver`: a takeover is a latch. Whenever the robot leaves the control mode that obeys the
  driver after it was seen in it, moving or not (the handheld, or `manual_mode`; with `0x21`
  `non_manual_mode` and the velocity source services), nothing moves the robot and nothing
  switches it back until the robot reports that mode again (handed back on the handheld, or the
  target mode service). Before, a 0.5 s pause of the autonomy re-armed the driver, which then
  switched the mode back, and a takeover while `cmd_vel` was zero was not seen.
- `x30_driver`: `sit_down`, `emergency_stop` and `stop_stepping` hold until `stand_up` or
  `start_stepping` (before: until `cmd_vel` rested for 0.5 s).

### Changed
- `x30_driver`: the automatic mode switch and stepping toggle happen only at the start of an
  episode (the first motion command after `cmd_vel` was quiet for `cmd_timeout_sec +
  stop_burst_sec`): the mode on a fresh report only, 3 tries 0.5 s apart; stepping from
  torque-controlled standing, 3 tries through the toggle guard; never again in that episode.
- `x30_driver`: after a takeover, a hold or an emergency stop or fall ends, motion goes out only
  once `cmd_vel` has been zero (one message) or quiet (`cmd_timeout_sec`); this now applies to
  the target mode service too, which no longer re-arms at once. No timer re-arms anything.
- `x30_driver`: zeros go out for `stop_burst_sec` after the last motion or after the gate closed,
  then nothing, even while zero Twists keep arriving (axis and topic mode).
- `x30_driver`: a `cmd_vel` with NaN or inf in any of its six fields is a stop (zeros), with a
  throttled warning; before, only that component was dropped.
- `x30_driver`: `state` shows the gate as `velocity` (`driving`, `idle`, `hold:<service>`,
  `takeover (<mode> mode)`, `disarmed:<reason>`, `waiting for zero or quiet cmd_vel`) and who
  took over as `takeover`, replacing `armed` and `disarmed_by`; WARN while motion is asked for
  and held back.

### Removed
- `x30_driver`: the parameters `mode_refresh_sec` and `stepping_retry_sec`.

## [0.2.0] - 2026-10-18

### Fixed (safety)
- `x30_driver`: an operator takeover is no longer reverted. When the robot leaves the control
  mode that obeys the driver while `cmd_vel` moves it (manual on the handheld, or the
  `manual_mode` service), the driver stops switching and sends only zeros until `cmd_vel` has
  been quiet or zero for `stop_burst_sec`.
- `x30_driver`: `stop_stepping`, `sit_down` and `emergency_stop` are no longer undone by the
  next `cmd_vel` (the automatic stepping toggle would start stepping again): the velocity stream
  holds until `cmd_vel` rests, and holds during an emergency stop or fall.
- `x30_driver`: `height_normal` / `height_crawl` fail when the body height report still shows
  the old height (they succeed only without any height report); crawl needs a fresh motion state.

### Changed
- `x30_driver`: the automatic mode switch follows `command_prefix`: non-manual mode for `0x31`,
  manual mode for `0x21` (the robot obeys the handheld's source there). `auto_non_manual_mode` is
  now `auto_control_mode`. The simulated robot obeys `0x31` axes in non-manual and `0x21` axes in
  manual mode only.
- `x30_driver`: Run, Silent and unknown gaits get zeros instead of Walk's scaling, unless the new
  `gait_limits` parameter gives their speeds (it can also replace Appendix B gaits).
- `x30_driver`: velocities below 0.001 m/s (rad/s) are zero: noise no longer goes out as the ±656
  dead zone step.
- `x30_driver`: stand/sit and stepping toggles, from services and the automatic switch, go out
  only once the reported state shows the previous one; posture services run one at a time.
- `x30_driver`: `emergency_stop` sends the soft emergency stop three times and succeeds once the
  robot reports emergency stop or fall.
- `x30_driver`: `save_data` only while sitting or after an emergency stop, and with the source
  prefix (`0x31010C01`); `18` still goes out as it is.
- `x30_driver`: axis commands need a fresh `0x1009` motion state report.
- `x30_driver`: the elevation services leave multi-frame mode as 1.5.2.5 says (new mode, then
  the LiDAR-inertial odometry off, `lio_port`); the velocity source services set non-manual mode
  first (1.5.1.1); the obstacle height code is a parameter (`obstacle_height_code`).
- `x30_driver`: `command_prefix` other than `0x21` / `0x31` stops the node; `heartbeat_hz` is at
  least 2 Hz; shutdown stops the stream before sending zero axes three times.
- `x30_driver`: `state` shows `armed`, `disarmed_by`, `obeyed_in`; tests run the velocity stream
  in a closed loop with the simulated robot and the node with rospy stand-ins.

## [0.1.0] - 2026-10-18

### Added
- `x30_driver`: the motion host's UDP protocol (Jueying X30 API Documentation V1.0.5-0).
  Heartbeat and connect; `cmd_vel` as axis commands scaled by the current gait's speeds
  (Appendix B), in the stepping state only, with automatic non-manual mode and stepping and a
  0.5 s watchdog; `velocity_mode: topic` for `/cmd_vel_corrected`. Reports `0x1008`, `0x1009`,
  `0x100A`, `0x100B`, `0x21050F0A` and `0x11050F08` as `odom`, `imu`, `joint_states`,
  `battery`, `state` and `diagnostics`, each struct recognised by its length (natural or
  pack(1), 8- or 4-byte `long`). Trigger services for posture, stepping, control mode, body
  height, every gait, the soft emergency stop and saving data, each checked against the robot's
  state and waiting for the reported change; services for the perception host's elevation map
  modes, obstacle avoidance, velocity source, obstacle height and autonomous charging.
  `command_prefix` (`0x31` autonomous module, `0x21` remote control) and `save_data_code`.
- `x30_driver`: `fake_robot.py`, a simulated motion and perception host for the tests and for
  trying the driver without a robot; unit tests against the document and a rostest of the node
  against the simulated robot.
- `x30_description`: Deep Robotics' official X30 model (deep_robotics_model `X30.urdf`, BSD-3)
  with its meshes as xacro: official link and joint names, four LiDAR frames, `base_link` root,
  `imu_link`; `description.launch`, `display.launch` and an RViz configuration.
- `x30_bringup`: one launch for the driver and the URDF.
- README with the network, a quick start, the interface and a checklist for the real robot; CI
  in a `ros:noetic-ros-base` container (build, unit tests, rostest, launch file checks,
  `check_urdf`).
