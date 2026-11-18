# NeuPAN patches for the X30

[NeuPAN](https://github.com/hanruihua/NeuPAN) (Han et al., T-RO 2025) plans in one MPC both the
tracking of a path and the avoidance of raw obstacle points, the footprint's distance to each
point coming from DUNE, a small network trained once per footprint. x30_navigation's `neupan`
controller runs it; [`install_neupan.sh`](../install_neupan.sh) installs it from these files.

| Patch | |
|---|---|
| `0001-feat-holonomic-kinematics-...` | the `holonomic` kinematics: forward and sideways speed in the robot frame and the yaw rate, a heading NeuPAN tracks (its `omni` keeps the heading and moves at a speed and a direction) |
| `0002-fix-package-the-neupan-modules` | `pyproject.toml` lists the `neupan` package, so a wheel holds it (with `py-modules` it held nothing) |

They apply to the `py38` branch at `70991f0` (Python 3.8, Noetic), pinned in the install script.
`holonomic` adds a third control value: the state stays (x, y, yaw), the MPC's controls are
(vx, vy, w) with the dynamics linearized at the nominal heading and speeds, the speed cost stays
on vx, `lateral_weight` (robot argument, default 1.0) prices vy so the robot walks forward unless
stepping aside pays, and the state cost keeps the yaw (q_s as three weights). `max_speed` and
`max_acce` take three values; the other kinematics are unchanged.

```bash
git clone https://github.com/hanruihua/NeuPAN.git && cd NeuPAN
git checkout 70991f0d96b7a15135ba02ea8ad3f3090ff0926e
git apply ../x30_ros/tools/neupan/*.patch   # or git am, to keep the commits
```

## License

NeuPAN is GPL-3.0, and so are these patches (derived from it): this folder is not under
x30_ros's Apache-2.0. x30_ros holds no NeuPAN code; its controller imports NeuPAN from where the
install script put it, at run time.
