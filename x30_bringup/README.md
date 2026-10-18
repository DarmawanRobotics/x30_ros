# x30_bringup

The Jueying X30 in one launch: [x30_driver](../x30_driver/README.md) and
`robot_state_publisher` with the URDF of [x30_description](../x30_description/README.md), in one
namespace.

## Usage

```bash
roslaunch x30_bringup bringup.launch                                  # namespace x30
roslaunch x30_bringup bringup.launch params_file:=$HOME/x30_driver_params.yaml rviz:=true
```

## Launch

`launch/bringup.launch`:

| Argument | Default | |
|---|---|---|
| `namespace` | `x30` | driver topics and services, `robot_description` |
| `params_file` | [`x30_driver/config/driver_params.yaml`](../x30_driver/config/driver_params.yaml) | a copy with this robot's settings, e.g. `invert_wz` |
| `overrides_file` | empty | a few parameters on top, e.g. `x30_driver/config/fake_robot.yaml` |
| `rviz` | `false` | RViz with the robot model |

## Nodes

| Node | Namespace | |
|---|---|---|
| `x30_driver` | `x30` | `cmd_vel` in; `odom`, `imu`, `joint_states`, `battery`, `state`, `diagnostics`; Trigger services |
| `robot_state_publisher` | `x30` | TF `base_link` → `TORSO` → legs, `imu_link`, `LIDAR_*` from `joint_states` |
| `rviz` (`rviz:=true`) | `x30` | |

The driver publishes no TF by default (`publish_tf: false`): `odom` → `base_link` belongs to
the odometry or localization in use.
