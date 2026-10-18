# Changelog

All notable changes to x30_ros. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

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
