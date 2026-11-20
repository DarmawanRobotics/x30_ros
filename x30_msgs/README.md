# x30_msgs

Messages and services of the X30 stack, the ROS Noetic port of
[dharma_interfaces](https://github.com/DarmawanRobotics/dharma_interfaces): every package of the
stack talks through these types. Definitions: [msg/](msg), [srv/](srv).

```bash
rosmsg show x30_msgs/ControlStatus
```

## Interfaces

**Status** (latched, enum constants in the message)

| Message | Topic | |
|---|---|---|
| `OdometryStatus` | `odometry/status` | INITIALIZING, OK, DEGRADED, LOST; estimator speed, tilt while still |
| `TripStatus` | `odometry/trip` | distance, moving time, current, average and top speed since `odometry/trip_reset` |
| `LocalizationStatus` | `localization/status` | NONE, LOCALIZED, UNCERTAIN; source of the last correction, 1-sigma uncertainty |
| `SiteStatus` | `site/status` | active site, sites on the robot, fiducials, routes (no map) |
| `NavigationStatus` | `navigation/status` | INACTIVE ... GOAL_REACHED / FAILED, pause reasons, progress, cross-track error |
| `DockingStatus` | `docking/status` | IDLE, SEARCHING ... DOCKED / FAILED, tag visibility, errors to the dock, attempt, charging |
| `ControlStatus` | `control/status` | AUTO / REMOTE, e-stop and its reasons, re-arm, protective field state, clearance, speed scale |
| `HealthStatus` | `health/status` | OK / WARN / ERROR, issues, speed scale requested from the control mux |
| `InspectionStatus` | `inspection/status` | IDLE, INSPECTING, DISABLED; points and targets of the site, cameras connected, the last point and its severity, the ambient temperature |

**Data**

| Message | |
|---|---|
| `SectorScan` | nearest obstacle per angular sector; `ranges <= BLOCKED` or NaN means blocked or unknown and is an obstacle at the robot |
| `AvoidanceCommand` | reactive avoidance: heading offset (rad), yaw-rate bias (rad/s), lateral bias, speed scale, emergency |
| `Waypoint`, `WaypointPath` | a path in the map frame (yaw in rad, `dwell_sec > 0`: stop point) |
| `StopPointEvent` | ARRIVED, INSPECTED, DEPARTED, SKIPPED at a stop point |
| `InspectionReport`, `InspectionFinding` | `inspection/report`: everything measured at an inspection point (DONE, PARTIAL, FAILED, NO_TARGETS), its findings (thermal criteria, partial discharge) with NORMAL ... CRITICAL and the evidence files |

**Services**

| Service | Server | |
|---|---|---|
| `SetPath`, `LoadPath` | `navigation/set_path`, `navigation/load_path` | give the follower a path or a taught route |
| `MarkStopPoint` | `route/mark_stop_point` | stop point at the current pose while teaching a route |
| `SetMode` | `control/set_mode` | `auto` or `remote` with the operator source |
| `ListSites`, `LoadSite` | `site/list`, `site/load` | the sites on the robot, the active one |
| `LoadFile` | `localization/load_tags` | switch to a file inside the sites directory |
| `AddTarget` | `inspection/add_target` | teach an inspection target from the PTZ camera's current view where the robot stands |

## Conventions

- SI units, radians, REP-103 / REP-105 frames; units are written next to every field.
- States are `uint8` constants, never free strings; `message` fields are for humans only.
- No `std_srvs/SetBool` for safety actions: a default request must never release or stop
  something, so these are separate `std_srvs/Trigger` services.
- Releases follow SemVer; a breaking change bumps the minor version before 1.0 and every package of
  the stack moves with it.

The types are the same as Dharma's (only `TripStatus.since` is the ROS 1 `time`), so code and
tools move between the two stacks unchanged.

x30_msgs is released under the [Apache License 2.0](../LICENSE).
