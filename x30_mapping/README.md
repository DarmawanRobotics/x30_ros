# x30_mapping

The sites of the X30, the ROS Noetic port of
[dharma_mapping](https://github.com/DarmawanRobotics/dharma_mapping) without its map: a site is
one directory per place, with the fiducials recorded there and the routes taught there, and
route teaching. Nothing maps: there is no occupancy grid, no `/map` and no mapping session.

## Sites

```
<sites_dir>/<site>/
├── tags.json                   fiducial poses in the map frame (localization/record_tags)
└── routes/<date>_<time>.csv    taught routes (x, y, yaw_deg, dwell_sec > 0: stop point)
```

`sites_dir` is `~/x30/map`, shared with x30_localization and x30_navigation. A site's map frame
is where the odometry started (localization's `assume_start_origin`: the dock, every time), so
its routes and fiducials are in that frame. Site names are `^[a-z0-9][a-z0-9_-]{0,47}$`; every
path is derived from `sites_dir` and a site name, never taken from a request. Broken site files
are notes in `site/status`, never a crash. `create_site` (on by default) creates the launch
`site` at start when it is missing: an empty folder for its routes and tags.

## Nodes

| Node | Role |
|---|---|
| `site_manager_node` | the active site and its files; hands localization the site's tags file |
| `route_recorder_node` | teach a route: samples map -> base_link while localized into the active site's `routes/` |

| Interface | Type | |
|---|---|---|
| `site/status` | `x30_msgs/SiteStatus`, latched | site, sites, tags, routes, notes |
| `route/saved` | `std_msgs/String` | path of a saved route |
| `site/list`, `site/load` | `ListSites`, `LoadSite` | another site resets localization |
| `route/start`, `route/stop` | `std_srvs/Trigger` | refused unless localization is `LOCALIZED` |
| `route/mark_stop_point` | `MarkStopPoint` | the current pose waits `dwell_sec` |

Used: `localization/status`, `localization/load_tags` (a path inside `sites_dir`),
`localization/reset`. The site manager calls them with a timeout (`service_timeout_sec`): rospy
has none per call, so a call that does not answer in time is a failure while it finishes in the
background.

Parameters: [config/mapping.yaml](config/mapping.yaml) (one section per node) on top of the
dataclass defaults in [core/config.py](src/x30_mapping/core/config.py); robot frames from the
robot profile (`robot/`).

## Launch

```bash
roslaunch x30_mapping mapping.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml site:=gardu-a
rosservice call /route/start                                       # teach a route, localized
rosservice call /route/mark_stop_point "{dwell_sec: 10.0}"
rosservice call /route/stop
```

| Argument | Default | |
|---|---|---|
| `params_file` | | overrides of config/mapping.yaml, the same layout |
| `robot_params` | | the robot profile, loaded as `robot/` |
| `sites_dir` | | the sites directory of both nodes |
| `site` | | the site at start |
| `create_site` | | create `site` at start when it is missing (the file's: `true`) |
| `respawn`, `output` | `true`, `screen` | |

Empty arguments override nothing.

## Tests

`catkin_make run_tests_x30_mapping`: routes and the site store (unit); both nodes on a private
rosmaster (routes only while localized with stop points, the site manager against fake
localization services: corrupt files at boot, the launch site created, site switches resetting
localization, no map and no mapping session); and the launch file's parameters.
