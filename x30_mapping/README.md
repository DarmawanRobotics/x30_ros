# x30_mapping

The site maps of the X30, the ROS Noetic port of
[dharma_mapping](https://github.com/DarmawanRobotics/dharma_mapping): occupancy maps recorded
from FAST-LIO's registered scans, sites (one directory per mapped place with its map, the
fiducials that anchor it and its routes) and route teaching.

## Sites

```
<sites_dir>/<site>/
├── map.yaml, map.pgm           occupancy map (map_server format)
├── map.yaml.bak, map.pgm.bak   the previous map (a loadable pair)
├── tags.json                   fiducial poses in the map frame (localization/record_tags)
├── routes/<date>_<time>.csv    taught routes (x, y, yaw_deg, dwell_sec > 0: stop point)
└── backup-<time>[-n]/          what a mapping reset archived
```

`sites_dir` is `~/x30/map`, shared with x30_localization. A site's map frame is the frame its
fiducials were recorded in. Site names are `^[a-z0-9][a-z0-9_-]{0,47}$`; every path is derived
from `sites_dir` and a site name, never taken from a request. The map pair is written crash safe
(temporary directory, fsync, rename; the YAML records the image size and CRC so a torn pair is
refused on load and the `.bak` pair is used). Broken site files are notes in `site/status`,
never a crash.

A site needs no map. `create_site` creates the launch `site` at start when it is missing, an
empty folder for its routes and tags: odometry only (x30_bringup's `odometry_only`), where the
map frame is where the odometry started. A robot that never maps (`grid:=false`, the X30's
navigation host) runs the site manager and the route recorder alone: no grid mapper, and the
site manager's `mapping: false` offers no `mapping/start`, `mapping/stop` or `mapping/save`.
`create_site` and `start_mapping` exclude each other.

## Nodes

| Node | Role |
|---|---|
| `grid_mapper_node` | log-odds occupancy grid of one site from `/cloud_registered_body` while mapping |
| `site_manager_node` | the active site and its files, the only publisher of `/map`; hands localization the site's tags file; mapping sessions |
| `route_recorder_node` | teach a route: samples map -> base_link while localized into the active site's `routes/` |

| Interface | Type | |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid`, latched | the site map, or the live grid while mapping |
| `mapping/grid` | `nav_msgs/OccupancyGrid`, latched | the live grid |
| `mapping/status` | `x30_msgs/MappingStatus`, latched | scans, `rejected` (jumps, corrections), area |
| `site/status` | `x30_msgs/SiteStatus`, latched | site, sites, has_map, mapping, tags, routes, notes |
| `route/saved` | `std_msgs/String` | path of a saved route |
| `site/list`, `site/load` | `ListSites`, `LoadSite` | another site resets localization |
| `mapping/start` | `StartMapping` | `reset`: archive the old site, start over |
| `mapping/stop`, `mapping/save` | `std_srvs/Trigger` | a stop the mapper does not confirm is a failure |
| `route/start`, `route/stop` | `std_srvs/Trigger` | refused unless localization is `LOCALIZED` |
| `route/mark_stop_point` | `MarkStopPoint` | the current pose waits `dwell_sec` |

Used: `localization/status`, `localization/load_tags` (a path inside `sites_dir`),
`localization/start`, `localization/reset`, `localization/assume_origin`. Internal:
`grid_mapper/start|stop|save|load` (site names only). The site manager calls them with a timeout
(`service_timeout_sec`): rospy has none per call, so a call that does not answer in time is a
failure while it finishes in the background.

## Mapping

- **Frames.** `mapping/start` on a new site (or with `reset`) maps in the current map frame, or
  takes the current pose as origin when not localized. Continuing a site that has a map or tags
  needs the robot localized in _its_ frame: on one of its tags or by a pose estimate on its map
  (an `origin` localization at boot does not count); otherwise `localization/start` must find a
  tag first.
- **Which scans.** Points between `scan/height_min` and `scan/height_max` above the floor,
  measured in the map frame (floor = base_link minus `robot/base_height`; localization levels
  odom with the IMU, so the band holds while the body pitches), minus the robot footprint. Scans
  taken while map -> odom changes (a correction in flight) are skipped; a pose jump beyond
  `gate/max_jump_m` + max speed × dt holds integration until a scan agrees with the map again.
  The grid never grows beyond `grid/max_size_m` per side (150 m: a larger substation needs more,
  36 MB per 150 m square at 5 cm).
- **Poses.** Scans are integrated at the TF pose. `refine/enabled` matches each scan on the
  current grid first (gated on score and ambiguity); it is off by default. There is no loop
  closure.

Parameters: [config/mapping.yaml](config/mapping.yaml) (one section per node) on top of the
dataclass defaults in [core/config.py](src/x30_mapping/core/config.py); robot geometry and frames
from the robot profile (`robot/`).

## Launch

```bash
roslaunch x30_mapping mapping.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rosservice call /mapping/start "{site: gardu-a, reset: false}"   # walk the site
rosservice call /mapping/save && rosservice call /mapping/stop
rosservice call /route/start                                       # teach a route, localized
rosservice call /route/mark_stop_point "{dwell_sec: 10.0}"
rosservice call /route/stop
```

| Argument | Default | |
|---|---|---|
| `params_file` | | overrides of config/mapping.yaml, the same layout |
| `robot_params` | | the robot profile, loaded as `robot/` |
| `sites_dir` | | the sites directory of the three nodes |
| `site`, `start_mapping` | | the site at start; map it at start when it is new |
| `create_site` | | create `site` at start when it is missing, without a map (odometry only) |
| `grid` | `true` | `grid_mapper_node` and the mapping sessions; `false`: a robot without a map |
| `respawn`, `output` | `true`, `screen` | |

Empty arguments override nothing.

## Tests

`catkin_make run_tests_x30_mapping`: the grid, the integrator, map files, routes, sessions and
the site store (unit); the three nodes on a private rosmaster (scans through TF into a grid,
saving only into the mapped site, routes only while localized with stop points, the site manager
against fake localization and mapper services: ordering, archives, a timed-out stop, corrupt
files at boot, a site created without a map); and the launch file's parameters.
