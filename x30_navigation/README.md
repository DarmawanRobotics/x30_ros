# x30_navigation

Where the X30 walks to, the ROS Noetic port of
[dharma_navigation](https://github.com/DarmawanRobotics/dharma_navigation): a planner on the site
map, routes from the active site, a path follower with pluggable controllers, reactive
avoidance and tag-based docking. Every command goes to the control mux
(`control/nav_cmd_vel`, x30_control), never to the robot's `cmd_vel`: the mux's protective field
and e-stop always apply.

## Nodes

| Node | Role |
|---|---|
| `planner_node` | `planner/go_to {x, y, yaw}`: plan on `/map` with a footprint costmap, hand the path to the follower (`navigation/set_path`, `navigation/start`), replan around blockages; refused for a site without a map (an empty `/map`) |
| `path_loader_node` | `navigation/load_path`: read a route CSV inside `sites_dir`, resample and smooth it (stop points never move), publish it latched |
| `path_follower_node` | follow the route: join it, follow, final x / y / yaw approach, stop points with inspection hooks, pause reasons; publishes `control/nav_cmd_vel` |
| `obstacle_avoidance_node` | a reactive policy on `perception/sector_scan` around the direction of travel → `navigation/avoidance` (heading offset, speed scale, emergency stop) |
| `docking_node` | `docking/dock`: centre on the dock tag (TF), walk to the docked pose, settle, sit and start charging (robot hooks); `docking/undock`, `docking/cancel` |

```
planner/go_to ─► planner_node ─► navigation/set_path ─┐
navigation/load_path ─► path_loader_node ─► navigation/waypoints ─┤
                                                      ▼
perception/safety_scan, navigation/avoidance ─► path_follower_node ─► control/nav_cmd_vel ─► control_mux_node ─► /x30/cmd_vel
control/status, localization/status, /x30/battery, /x30/odom ─┘
```

| Interface | Type | |
|---|---|---|
| `planner/go_to` | `x30_msgs/GoTo` | plan to x, y, yaw (map frame), load it and walk |
| `planner/plan` | `x30_msgs/PlanPath` | plan only |
| `planner/cancel` | `std_srvs/Trigger` | stop walking (never waits for a plan in progress) |
| `planner/status`, `planner/path`, `planner/costmap` | `PlannerStatus`, `Path`, `OccupancyGrid` (latched) | state, remaining, replans; plan; costmap (`publish_costmap`) |
| `navigation/set_path` | `x30_msgs/SetPath` | give the follower a path |
| `navigation/start`, `stop`, `pause`, `resume` | `std_srvs/Trigger` | run, stop (zero), hold, resume |
| `navigation/load_path` | `x30_msgs/LoadPath` | load a route CSV inside `sites_dir` |
| `navigation/waypoints`, `navigation/path` | `WaypointPath`, `Path` (latched) | the loaded route |
| `navigation/status` | `NavigationStatus` (latched) | state, pause reasons, index, count, remaining, cross track |
| `navigation/stop_point` | `StopPointEvent` | arrived / inspected / departed / skipped |
| `navigation/avoidance` | `AvoidanceCommand` | avoidance node → follower |
| `control/nav_cmd_vel` | `geometry_msgs/Twist` | the follower's (and docking's) command, to the control mux |
| `docking/dock`, `docking/undock`, `docking/cancel` | `std_srvs/Trigger` | dock and charge, stop charging and leave, stop |
| `docking/status`, `docking/goal` | `DockingStatus` (latched), `PoseStamped` | state, errors to the dock, attempt, charging; docked pose |

In: `/map`, `perception/safety_scan`, `perception/sector_scan`, `control/status`,
`localization/status`, the robot's battery and odometry (`battery_topic`, `odom_topic`).

## On rospy

The nodes are Dharma's, on `X30Node`: their callbacks run one at a time. Where Dharma ran a
callback beside the others (a separate callback group), it is a concurrent service here:

- `navigation/start` waits for the robot pose (`pose_wait_sec`) while the follower's tick and
  inputs keep running;
- `planner/go_to` and `planner/plan` wait for the follower's services beside the planner's
  subscriptions and status, one at a time with its monitor; `planner/cancel` never waits for
  them;
- hooks (start, stop, inspection, docking) are asynchronous calls with a timeout whose answers
  come back as the node's callbacks.

The follower checks once that its guard is at least as cautious as the control mux's field: it
reads the mux's `field/*` from the parameter server, where every X30Node writes the values it
runs with (the margins, the person's speed and the turning room). The guard checks what the
field checks, a turning translation along its arc included (one function of x30_utils.safety
for both), and it checks what is published: the ramps of the command shaper are comfort and
never outrun it, and a ramp bent toward something goes the guarded way. A turn that would sweep
into a point goes on straight and turns later. So the field only stands by: the mux never cuts
what the follower sends (`test_guard_field.py` drives the follower through the field, and puts
the field after the guard in 2000 random scenes of points close by and hard turns), where it
used to limit NeuPAN's passes and stop the robot in the pocket of the detour tests.

## Docking on the X30

`docking_node` docks onto a charger marked by a fiducial tag (`tag/frame`, default
`charging_dock`, on TF under the camera, e.g. x30_perception's ArUco detector) and calls the
robot's hooks: `hooks/sit`, `hooks/charge`, `hooks/stop_charge`, `hooks/stand` (lists of
`std_srvs/Trigger` services). On the X30 they are empty by default: x30_driver offers
`sit_down`, `stand_up` and the perception host's autonomous charging (`charge_start`,
`charge_stop`), whose own procedure walks the robot onto its charging pile, so which steps the
tag approach should take over has to be decided on the robot. Two values to set there:

- `goal/distance`: base_link from the dock tag where the robot stops (1.0 m by default, safe; the
  X30's front is 0.50 m from base_link);
- `charge/confirm_timeout_sec`: 0 (off) until x30_driver's battery reports a charging status.

## Parameters

[config/navigation.yaml](config/navigation.yaml) has the X30's values, one section per node, on
top of the dataclass defaults in [core/config](src/x30_navigation/core/config). The robot's
footprint, limits and frames come from the robot profile (`robot/`). The path controllers
(`holonomic`, `pure_pursuit`, `stanley`, `lqr`, `mppi`, `pid`, `neupan`) and the avoidance policies
(`vfh`, `braitenberg`, `potential_field`, `follow_gap`) each have their own section
(`<name>/...`); `controller` and `policy` switch them live. Every node validates its values at
start and applies live changes within a second (all or none; read-only values until restart).

## NeuPAN

`controller: neupan` runs [NeuPAN](https://github.com/hanruihua/NeuPAN) (Han et al., T-RO
2025): one MPC that tracks the route ahead and keeps the footprint off the safety scan's points
directly, no costmap and no map (odometry only works). DUNE, a small network trained for the
footprint, gives the distance from the body to every point at every planned pose. On the X30 the
MPC plans the forward and sideways speed and the yaw rate: the `holonomic` kinematics patched
into NeuPAN for the X30 ([tools/neupan](../tools/neupan)), so the body turns with the route and
steps aside when that pays; a robot profile that cannot strafe plans with NeuPAN's `diff`.

```bash
bash tools/install_neupan.sh                    # once: a venv in ~/x30_ws/neupan (internet)
export X30_PYTHON=~/x30_ws/neupan/bin/python     # the path follower runs with it
roslaunch x30_bringup bringup.launch navigation:=true controller:=neupan ...
```

- **It keeps its own clearance**: reactive avoidance stands by. Path shift (`avoidance_mode:
  path_shift`, the default) still picks the side to pass an obstacle on: NeuPAN alone stands
  before one right on the route, with no side to prefer, until the follower's detour. The
  follower's guard, its BLOCKED state (the planner replans, the detours) and the control mux's
  protective field stay as they are and keep the last word.
- **Its model**: [models/dune_x30.pth](models/dune_x30.pth), trained for the X30's footprint
  (1000 x 470 mm) on points up to 8 m away, with its record
  [dune_x30.json](models/dune_x30.json): the footprint, the training, 1.5 mm mean distance error,
  at most 8 mm too far (the errors near the corners are on the safe side). The controller
  refuses a model of another footprint: after changing `robot/footprint`, train one (about
  30 min on one core) and give it as `dune_checkpoint:=` (navigation.launch) or
  `neupan/dune_checkpoint`:

  ```bash
  $X30_PYTHON $(rospack find x30_navigation)/scripts/train_dune \
      --robot-params $(rospack find x30_bringup)/config/robot.yaml --out ~/x30/dune_mine.pth
  ```

- **Without NeuPAN** (no venv, `X30_PYTHON` unset, or NeuPAN without the patches) the follower
  does not start with `controller: neupan` and says how to install it; the other controllers
  never need it.
- **Timing**: one solve every `step_sec` (0.1 s), 15 to 40 ms on one core; between solves the
  follower keeps the plan's first command. All its parameters are read at start. The latest
  solve is in `navigation/status` (`lap 1 | neupan 23 ms | ...`); when more than one solve in
  ten of the last 50 takes longer than the step, the follower logs a warning (every 30 s).
  `bench_neupan` times the solves on a computer before the robot walks: the follower's
  controller from the robot profile and navigation.yaml on an open route and in a corridor past
  a box (a few seconds, the robot never moves; exit 1 when the slow ones overrun the step):

  ```bash
  $X30_PYTHON $(rospack find x30_navigation)/scripts/bench_neupan     # --threads 2 to compare
  ```

| `neupan/...` | X30 | |
|---|---|---|
| `obstacle_range` | 4.0 m | the safety scan's points it plans around (DUNE's range minus what the horizon travels) |
| `max_clearance` | 0.25 m | clearance it seeks; a passage narrower than the body plus twice this stalls it |
| `min_clearance` | 0.1 m | clearance it keeps from every point |
| `weight_speed` | 2.0 | the speed error's weight: below 2 it rather stands before an obstacle than walks around it |
| `weight_xy`, `weight_yaw`, `weight_lateral` | 1.0, 0.5, 1.0 | position, heading and sideways walking costs |
| `horizon_steps`, `step_sec` | 10, 0.1 s | the horizon: 1 s ahead |
| `stop_distance` | 0.05 m | a point this near the body: it stands |
| `max_reverse_speed` | 0.15 m/s | backward speed cap |

## Launch

```bash
roslaunch x30_navigation navigation.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rosservice call /navigation/load_path "{path: gardu-a/routes/patrol.csv}"
rosservice call /navigation/start
rosservice call /planner/go_to "{x: 8.0, y: 4.0, yaw: 1.57}"
```

| Argument | Default | |
|---|---|---|
| `params_file` | | overrides of config/navigation.yaml, the same layout |
| `robot_params` | | the robot profile, loaded as `robot/` |
| `battery_topic`, `odom_topic` | `/x30/battery`, `/x30/odom` | x30_driver's battery and leg odometry |
| `sites_dir`, `route` | | `path_loader_node`'s, when set |
| `docking` | `true` | start `docking_node` |
| `planner` | `true` | start `planner_node`; `false` for a robot without a map (it plans on one) |
| `controller` | | the path follower's, when set (else navigation.yaml's `holonomic`) |
| `dune_checkpoint` | `models/dune_x30.pth` | NeuPAN's DUNE model (a `params_file` may give another) |
| `python` | `$X30_PYTHON` | the path follower's interpreter: the NeuPAN venv |
| `respawn`, `output` | `true`, `screen` | |

## Tests

`catkin_make run_tests_x30_navigation`: the planner, the follower (controllers, path shifting,
backtracking, stop points), avoidance policies, docking and path processing (unit, 277); the five
nodes on a private rosmaster (following with pauses, e-stop and re-arm, start waiting for the
pose, cancel never blocked by a slow send, replanning and a blind scan, routes inside the sites
directory, docking with its hooks, a clean SIGINT while ticking); and the launch file's
parameters and remaps. NeuPAN's checks and its model's record always; its driving (a pole on
the route, a 1 m passage, a wall across, the turn of an L, a robot without strafing) needs NeuPAN:
`$X30_PYTHON -m pytest test/unit/test_neupan.py test/unit/test_dune_model.py`, as CI's `neupan`
job does along with x30_bringup's `stack_neupan.test` (the patrol loop on x30_sim).
