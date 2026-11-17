# x30_control

The control mux of the X30 stack, the ROS Noetic port of
[dharma_control](https://github.com/DarmawanRobotics/dharma_navigation/tree/main/dharma_control):
the **only** node that publishes the robot's `cmd_vel` (`/x30/cmd_vel`, x30_driver). It
arbitrates navigation (AUTO) and one operator source (REMOTE), keeps every command inside the
robot's speed envelope, limits it with a protective field on the 360° safety scan (translation
**and** rotation) and holds a latched e-stop with a hardware input and re-arm after release.

## Safety architecture: two channels

This node is **channel 2**, a software safety function. It is not certified and must never be
the only protection of a robot that drives among people.

| Channel | What | Stops the robot by |
|---|---|---|
| 1 (hardware) | e-stop buttons and, where required, a safety-rated scanner or bumper wired to the motor driver's safe torque off / brake input | cutting the drive, independently of any software |
| 2 (this node) | speed envelope, protective field, watchdogs, latched software e-stop | publishing zero on `cmd_vel` (and streaming zero while engaged) |

Wire channel 1's state to `control/hw_estop` (Bool, true = pressed) so channel 2 latches too:
while it is pressed the software e-stop cannot be released, and after the button is released an
operator still has to release the software e-stop explicitly. With `hw_estop_timeout_sec > 0`
a silent `control/hw_estop` publisher engages the e-stop (the signal is then required).

## Pipeline

```
control/nav_cmd_vel ─────────┐
control/remote/<source>/cmd_vel ─► arbitration ─► envelope ─► health scale ─► field ─► cmd_vel
                  (mode, watchdog, e-stop latch, re-arm)                   (translation, rotation,
                                                                            combined arc, stale/blind)
```

- **Arbitration.** AUTO forwards `control/nav_cmd_vel`, REMOTE forwards
  `control/remote/<source>/cmd_vel` of the selected source only (`remote_sources`, default
  `[handheld, dashboard]`). `control/set_mode` with the current mode and source is a no-op; a
  change publishes one zero. Unknown modes or sources are rejected.
- **Watchdog.** The active source silent for `nav_timeout_sec` / `remote_timeout_sec` gets one
  zero. All timeouts and the e-stop hold run on the **steady clock**: clock jumps and
  `use_sim_time` never disable them.
- **E-stop.** Engaged by `control/estop/engage`, `control/estop_request` (String reason),
  `control/hw_estop` or an internal error. Reasons accumulate (oldest first, no duplicates).
  Zero is published at once and streamed at `estop_hold_hz`. Mode switches are allowed while
  engaged but forward nothing. `control/estop/release` is refused while the hardware e-stop is
  pressed.
- **Re-arm.** At start, after a release and after switching to a remote source, the source is
  disarmed: it must send `rearm_neutral_count` consecutive neutral commands (every axis below
  `neutral_eps`) before anything it sends is forwarded, so a stick held during a release never
  moves the robot. Navigation also re-arms by staying silent for `nav_timeout_sec`. The mux never
  calls navigation services: the follower reads `control/status` and pauses itself (`estop` is a
  sticky pause reason there, `remote` a transient one, `disarmed` while `armed` is false: it
  then sends neutral commands, so a follower started before, right after or across a respawn of
  the mux is armed and drives).
- **Envelope.** `|vx| <= max_vx`, `|vy| <= max_vy` (0 for `diff` kinematics), `|wz| <= max_wz`
  from `robot.limits`; x and y are scaled together (same heading). Any non-finite axis makes the
  command zero. With `use_health_limit`, every axis is scaled by `health/status` `speed_scale`
  (stale health: no change).
- **Protective field.** AUTO always, REMOTE when `field.in_remote` (default true, with
  `field.remote_margin`). One braking model from `x30_utils.safety`, `d = v t_r + v²/2a`,
  plus the separation term of ISO/TS 15066 speed and separation monitoring: a person walking
  toward the robot at `field.human_speed` covers `v_h (t_r + v / a)` until it stands:

  - translation: free distance of the footprint swept along the motion at its own orientation,
    widened across it by `lateral_margin` (`core/sweep.py`; a point already beside the body
    within the margin may be passed, not approached) minus the margin bounds the speed so that
    `v t_r + v²/2a + v_h (t_r + v/a)` fits; the follower guard uses the same file;
  - rotation: free turning angle of the footprint (`rotation_room`) minus `yaw_margin` bounds
    `|wz|` with `max_yaw_accel`;
  - combined motion: the footprint is checked along the commanded arc until standstill (reaction
    time, then proportional braking; the margin grown by the person's distance) and the command
    is scaled, keeping its curvature, until the arc is clear.

  `field.human_speed` defaults to 0.5 m/s: the static model (0) let a person walking at
  0.2 m/s brush the robot in the evaluation; 0.5 m/s covers slow walkers with a margin and keeps
  the robot able to pass people and reach stop points near objects. ISO 13855 assumes 1.6 m/s
  for a person walking toward a machine; use it where people may hurry toward the robot. The
  follower's `safety.human_speed`, `safety.lateral_margin` and `safety.pass_margin` must not
  be below the field's `human_speed`, `lateral_margin` and `margin`.

- **Scan validation.** `perception/safety_scan` is accepted only in `robot.frames.base` with
  full-circle coverage; it is STALE when invalid, not received for `field.scan_timeout_sec`
  (monotonic) or its stamp is older than `field.scan_max_age_sec` (or more than
  `field.scan_clock_tolerance_sec` in the future). Any blocked or NaN sector makes it BLIND.
  AUTO: STALE or BLIND -> zero. REMOTE: the fastest point of the body is capped at
  `field.remote_degraded_speed`; with a partly blind scan, translation toward a blocked sector
  is refused (it is an obstacle at the robot).
- **Fail-safe.** An exception in any callback engages the e-stop (`internal error`) and
  publishes zero; on shutdown the node publishes zero three times. Safety parameters are
  read-only at runtime and validated at start: the node refuses to start on a bad value.

## Topics and services

| Interface | Type | |
|---|---|---|
| `cmd_vel` (remapped to `cmd_vel_topic`, `/x30/cmd_vel`) | `geometry_msgs/Twist` | out: the robot's command (the only publisher) |
| `control/status` | `x30_msgs/ControlStatus`, latched | mode, remote source, e-stop and reasons, armed, field state, clearance, speed scale; on change and at `status_hz` |
| `control/nav_cmd_vel` | `geometry_msgs/Twist` | in: navigation (AUTO) |
| `control/remote/<source>/cmd_vel` | `geometry_msgs/Twist` | in: one per `remote_sources` entry (REMOTE) |
| `perception/safety_scan` | `x30_msgs/SectorScan` | in: 360° scan in the base frame (x30_perception) |
| `control/estop_request` | `std_msgs/String` | in: engage with a reason |
| `control/hw_estop` | `std_msgs/Bool` | in: hardware e-stop state, true = pressed |
| `health/status` | `x30_msgs/HealthStatus` | in, with `use_health_limit` |
| `control/set_mode` | `x30_msgs/SetMode` | `auto`, or `remote` with a source (empty: the first) |
| `control/estop/engage`, `control/estop/release` | `std_srvs/Trigger` | engage; release (refused while the hardware e-stop is pressed) |

The X30 has its own channel 1: the hard emergency stop at the tail cuts the joints, and the
handheld and the soft emergency stop button always work, past this node and past x30_driver.

## Parameters

Private parameters of `control_mux_node`, all read-only at runtime (a change on the server is
refused with a warning until a restart) and validated at start: the node refuses to start on a
bad value. Defaults and docs in [config/control.yaml](config/control.yaml).

| Parameter | Default | |
|---|---|---|
| `mode` | `auto` | mode at start |
| `remote_sources` | `[handheld, dashboard]` | operator sources |
| `nav_timeout_sec`, `remote_timeout_sec` | 0.5, 0.3 s | watchdog per source |
| `estop_hold_hz` | 10 | zero stream while engaged |
| `rearm_neutral_count`, `neutral_eps` | 3, 0.01 | re-arm after a release |
| `hw_estop_timeout_sec` | 0 (optional input) | engage when `control/hw_estop` is silent |
| `use_health_limit`, `health_timeout_sec` | false, 3 s | health speed scale |
| `status_hz` | 2 | `control/status` period |
| `field/in_remote` | true | field in REMOTE |
| `field/margin`, `field/remote_margin` | 0.15, 0.10 m | gap kept to obstacles |
| `field/lateral_margin`, `field/yaw_margin` | 0.10 m, 0.10 rad | swept path width, turning margin |
| `field/human_speed` | 0.5 m/s | assumed speed of a person walking toward the robot (ISO/TS 15066 separation) |
| `field/min_speed`, `field/min_yaw_rate` | 0.02 m/s, 0.05 rad/s | slower allowed motion stops |
| `field/remote_degraded_speed` | 0.15 m/s | REMOTE with a stale or blind scan |
| `field/scan_timeout_sec`, `field/scan_max_age_sec`, `field/scan_clock_tolerance_sec` | 0.5, 0.5, 0.1 s | scan freshness |
| `robot/*` | the robot profile | footprint, `limits` (max_vx, max_vy, max_wz, max_decel, max_yaw_accel, reaction_time), kinematics, `frames/base` |

Measure `robot/limits/max_decel`, `max_yaw_accel` and `reaction_time` on the robot (worst case:
floor, payload, battery) before it drives on its own.

## Launch

```bash
roslaunch x30_control control.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rosservice call /control/set_mode "{mode: remote, source: handheld}"
rosservice call /control/estop/engage
rosservice call /control/estop/release
```

Arguments: `cmd_vel_topic` (`/x30/cmd_vel`), `params_file` (overrides), `robot_params` (the
robot profile, loaded as `robot/`; empty: already on the server), `respawn`, `output`.

## Test

```bash
catkin_make run_tests_x30_control && catkin_test_results build/test_results/x30_control
```

The core (arbitration, envelope, field, sweep, scan validation) is tested without ROS; the node
runs in the test process against a private master (forwarding, e-stop and re-arm, hardware
e-stop, field stop and rotation limit, blind and stale scans, envelope, health limit, read-only
parameters, bad values, the fail-safe, shutdown zeros), and once in its own process with
simulated time and a stopped `/clock` (watchdog and hold on steady time).

x30_control is released under the [Apache License 2.0](../LICENSE).
