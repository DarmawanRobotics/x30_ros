# Changelog

All notable changes to x30_ros. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

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
