# x30_description

URDF of the Deep Robotics Jueying X30: Deep Robotics' own model with its meshes and LiDAR
frames, as xacro, with a `base_link` root and an `imu_link`.

## Usage

```bash
roslaunch x30_description display.launch                 # RViz with joint sliders (gui:=false: at rest)
roslaunch x30_description description.launch namespace:=x30
```

## Launch

| File | | Arguments |
|---|---|---|
| `description.launch` | `robot_description` (xacro) and `robot_state_publisher` in the namespace; the legs follow `<namespace>/joint_states` from x30_driver | `namespace` (`x30`), `model` (the xacro) |
| `display.launch` | the above, joint sliders and RViz ([rviz/x30.rviz](rviz/x30.rviz)) | `namespace`, `gui` (`true`) |

## Model

Links, joints, meshes, collisions and inertials are those of
[deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model) `X30/urdf/X30.urdf`
(BSD-3-Clause, [LICENSE.deep_robotics_model](LICENSE.deep_robotics_model)), unchanged and with
the official names: `TORSO`, `{FL,FR,HL,HR}_{HIP,THIGH,SHANK,FOOT}`, revolute joints
`*_HipX_joint`, `*_HipY_joint`, `*_Knee_joint` (the names x30_driver publishes), fixed
`*_Ankle`, and the four LiDAR links. A leg macro carries the per-leg inertials and mirrored
meshes of the model as they are; only the materials are added (the model has none), and its
MuJoCo compiler tag is left out.

## Frames (relative to `base_link`, metres, radians)

`base_link` is the centre of the body: x forward, y left, z up. `TORSO` hangs on it with an
identity joint.

| Frame | Position | Source |
|---|---|---|
| `*_HipX_joint` | (±0.291, ±0.08, 0), axis −x | model |
| `*_HipY_joint` | (0, ±0.11675, 0) from HipX, axis −y | model |
| `*_Knee_joint` | (0, 0, −0.30) from HipY, axis −y | model |
| `*_FOOT` | (0, 0, −0.31) from Knee, collision sphere r 0.036 | model |
| `LIDAR_FRONT_UP` | (0.44367, 0, 0.05415), pitch 0.4363 | model |
| `LIDAR_FRONT_DOWN` | (0.43297, 0, −0.05519), pitch 0.4363 | model |
| `LIDAR_REAR_UP` | (−0.46576, 0, 0.03484), pitch −0.8029 | model |
| `LIDAR_REAR_DOWN` | (−0.41074, 0, −0.05601), pitch −0.8029 | model |
| `imu_link` | (0, 0, 0) | **assumption**: the API documentation gives no IMU position |

Joint limits are the model's: HipX −0.323…0.585 rad on the left legs and −0.585…0.323 on the
right, HipY −2.967…0.262, Knee 0.349…2.531; effort 84 / 84 / 150 N m, velocity 17.5 / 17.5 /
16.1 rad/s. Total mass 55.8 kg.

No link may be called `body`, `odom`, `map` or `camera_init`: those frames belong to the
odometry and the navigation stack. A test checks it.

## Test

```bash
catkin_make run_tests_x30_description && catkin_test_results build/test_results
python3 -m pytest test            # with the xacro Python module, without ROS
```

The test expands the xacro, checks the legs, inertials, mirrored meshes and LiDAR frames against
the official model's numbers, that the revolute joints are x30_driver's `JOINT_NAMES`, that every
mesh is in the package, and the tree. CI also runs `check_urdf` on the expanded model.
