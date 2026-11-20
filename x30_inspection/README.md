# x30_inspection

Substation inspection at the stop points of the X30's patrol, the ROS Noetic port of HotDog's
inspection: the HIKMICRO PTZ camera ([hikmicro_driver](https://github.com/DarmawanRobotics/hikmicro_driver))
is pointed at each piece of equipment taught there and its temperatures are judged for
overheating, the Hertzino HZ-FA-371 ([hertzino_driver](https://github.com/DarmawanRobotics/hertzino_driver))
listens for partial discharge, and a report with the thermal, visible, sound map and PRPD images
is kept on the robot and published.

| Interface | Type | |
|---|---|---|
| `inspection/inspect` | `std_srvs/Trigger` | inspect the point where the robot stands (the follower's hook) |
| `inspection/add_target` | `x30_msgs/AddTarget` | teach a target from the camera's current pan, tilt and zoom |
| `inspection/report` | `x30_msgs/InspectionReport` | everything measured at one point, its highest severity, the folder |
| `inspection/status` | `x30_msgs/InspectionStatus`, latched | idle / inspecting / disabled, points, targets, both cameras connected, the last point and its severity, the ambient |
| in | `site/status`, `navigation/stop_point`, TF `map` → `base_link` | the active site; where the robot stopped (else its TF pose) |
| `/hikmicro/*` | hikmicro_driver | `ptz/move_to` (succeeds once the camera has arrived), `ptz/goto_preset`, `ptz/state`, `thermal/get_temperature`, `thermal/temperature`, `snapshot` |
| `/hertzino/*` | hertzino_driver | `detect` (one detection measured after the goal, its photos in the result), `status` |

The path follower calls `inspection/inspect` at every stop point and waits for the answer up to
`dwell + hooks/inspection_timeout_sec` (`x30_bringup inspection:=true` sets the hook, and 180 s
for a whole point). A stop with no inspection point within `match_radius_m` answers at once; a missing
or failing camera is reported, never raised, and the patrol goes on. One inspection runs at a
time; at shutdown the running one cancels its move or detection.

## Inspection points

`<sites_dir>/<site>/inspection.yaml`, next to the site's tags and routes (x30_navigation). A point
is where the robot stands, in the map frame, so every route stopping near it inspects it:

```yaml
points:
  - name: bay1_disconnector
    x: 12.40
    y: -3.10
    targets:
      - name: bay1_ds_clamps
        pan_deg: 35.0      # the camera's own pan (ptz/state), wrapped by the camera
        tilt_deg: -10.0    # the camera's tilt (-90 .. 90)
        zoom: 4.0          # 1 .. 60
        thermal: { enabled: true, strips: 3, max_temp_c: 0.0 }
      - name: bay1_ct_terminals
        pan_deg: -20.0
        tilt_deg: 5.0
        zoom: 2.0
        thermal:
          regions: # boxes in fractions of the thermal frame (0..1 from the top left), compared
            - { name: r, box: [0.12, 0.25, 0.31, 0.5] }
            - { name: s, box: [0.40, 0.25, 0.60, 0.5] }
            - { name: t, box: [0.69, 0.25, 0.88, 0.5] }
        acoustic: { enabled: true, duration_sec: 5.0 } # partial discharge, 5 s detection
```

**Teach** at the stop while the route is taught (the robot stands where it will stop) or while
the patrol dwells there: aim the camera (`/hikmicro/ptz/cmd_vel`, `ptz/move_to`, a preset), then

```bash
rosservice call /inspection/add_target "{name: bay1_ds_clamps, thermal: true, acoustic: true, strips: 3}"
```

The target goes to the point within `match_radius_m` (or a new point named after it, or the
`point` you name); the previous file is kept as `inspection.yaml.bak`. Edit the file for regions
or limits; it is read at every inspection.

## Overheat analysis

Per target, the camera moves there and the inspection waits until it has arrived (`ptz/move_to`
succeeds only then), `camera/settle_sec` more for the focus and a fresh thermometry frame. Each
region is then measured (`thermal/samples` times, the median kept) from the camera's
pixel-to-pixel thermometry: explicit `regions`, or `strips` side-by-side strips of the frame
(three phases), or the whole frame. The hottest region is judged by three criteria, each with
attention / warning / critical thresholds (degC), after the usual infrared-inspection practice
(NETA MTS):

| Criterion | Value | Default thresholds |
|---|---|---|
| `absolute` | the hottest temperature | 80 / 90 / 105 (or the target's `max_temp_c` as critical) |
| `ambient` | the hottest temperature over `thermal/ambient_c` | 11 / 21 / 41 |
| `similar` | the hottest region over the median of the others (two regions or more) | 2 / 4 / 16 |

`thermal/ambient_c` is live: set it to the air temperature of the day
(`rosparam set /inspection_node/thermal/ambient_c 33.0`). The full-resolution thermal and visible
JPEGs of the camera (`snapshot`) are the evidence, with the raw thermometry frame for analysis:
`<target>_thermal.npz`, the newest frame of `thermal/temperature` (hikmicro_driver's
`radiometric_rate`) taken once the camera settled, in degC per pixel with the camera's pan, tilt
and zoom (`x30_utils.io.read_thermal`, the format of x30_recorder). It is waited for up to
`camera/raw_wait_sec` (2.5 s); `camera/raw_thermal: false` keeps none.

## Partial discharge

For each acoustic target, after the robot stopped, one `detect` goal of hertzino_driver: the
detection time is the target's `duration_sec`, else `acoustic/duration_sec`; a detection already
running is restarted, so the result is always measured after the goal; the sound map and PRPD
photos come with the result and are saved next to the report. A camera in gas leak mode is
refused, before and after the detection. An acoustic-only target does not move the PTZ camera;
one with both moves it first, then detects.

| Type | Default severity | |
|---|---|---|
| `noise` | normal | no discharge |
| `corona` | attention | common on outdoor high-voltage parts: watch the trend |
| `surface` | warning | tracking on insulators, terminations |
| `floating` | critical | a floating (loose) part |

`acoustic/intensity_warning` / `intensity_critical` raise the severity by the sound map intensity
once the site's levels are known (0: off).

A target counts as measured when one of its checks measured: a camera failure with a detection
leaves the point `partial`, not `failed`.

## Reports

```
<reports_dir>/<YYYYMMDD>/<HHMMSS>_<point>/report.json, <target>_thermal.jpg, <target>_visible.jpg,
                                          <target>_thermal.npz, <target>_soundmap.jpg,
                                          <target>_prpd.jpg
<reports_dir>/<YYYYMMDD>/findings.csv           every finding of the day, one row each
```

`report.json` holds the point, the robot's position, the state (`done`, `partial`, `failed`,
`no_targets`), the highest severity and every finding with its value, reference and delta.
`inspection/report` carries the same, with the paths of the images and raw frames.

**Retention.** A robot on patrol fills its disk with reports, and a full disk writes nothing more:
reports, maps or routes. At start and after every report whole days go, never today's: those
older than `reports_keep_days` (0: kept), then the oldest while less than `reports_min_free_gb`
(6 GB, above x30_health's warning at 5 GB) is free, each removal a warning in the log. Copy the
reports off the robot (the command center) before they are this old; `reports_min_free_gb: 0`
keeps every one.

## Cameras

The drivers come from [x30.repos](../x30.repos); `x30_bringup hikmicro:=true hertzino:=true`
starts them at `hikmicro_host` (factory `192.168.1.64`) and `hertzino_host` (factory
`192.168.2.204`): give the robot computer an address in both subnets. Their passwords come from
`HIKMICRO_PASSWORD` and `HERTZINO_PASSWORD` in the environment, never from a parameter
(roslaunch prints the parameters). `camera/park_preset` calls a preset after each point (0: the
camera stays). `inspection/status` shows `thermal_connected` and `acoustic_connected`.

## Parameters

Defaults in [core/config.py](src/x30_inspection/core/config.py), the X30's in
[config/inspection.yaml](config/inspection.yaml). `sites_dir` and `reports_dir` default to
`~/x30/map` and `~/x30/reports`; x30_bringup passes its `sites_dir` and `reports_dir`
(`sim:=true`: `/tmp/x30_sim/sites`, `/tmp/x30_sim/reports`).

## Launch

```bash
roslaunch x30_inspection inspection.launch robot_params:=$(rospack find x30_bringup)/config/robot.yaml
rostopic echo /inspection/status
rosservice call /inspection/inspect
```

Arguments: `params_file`, `robot_params`, `sites_dir`, `reports_dir`, `respawn`, `output`.

## Tests

`catkin_make run_tests_x30_inspection`: the criteria, the plan file, the point sequence against a
fake camera (order, samples, deadline, cancel, failures), the reports, the discharge judgement and
the check around one detection, teaching (unit); the node against faked hikmicro_driver and
hertzino_driver on a private rosmaster: a stop at a point (report, evidence, the camera parked),
a stop elsewhere, missing drivers, a stalled camera, one inspection at a time, the status,
teaching from the camera's view; the launch file.
