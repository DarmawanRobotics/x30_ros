# x30_utils

The shared toolbox of the X30 stack, the ROS Noetic port of
[Dharma](https://github.com/DarmawanRobotics/Dharma): math, typed configuration, the robot
profile, one safety model, logging, file IO, grids, and `X30Node`, the rospy base class of every
node.

Everything outside `x30_utils.ros` and `x30_utils.testing` is plain Python and numpy (the same
code as `dharma_utils`), so the algorithms of every package are unit-tested without ROS. It runs
on Python 3.8 (Ubuntu 20.04): modules start with `from __future__ import annotations`, and
`test_style.py` rejects what Python 3.8 cannot run.

## Modules

| Module | |
|---|---|
| `math` | angles, quaternions (x, y, z, w), SE(3) 4x4 transforms, `Pose2D` (compose with `@`), filters (`Ema`, `SlewLimiter`, `RateMeter`, `Cusum`, `Hold`), NaN-safe `clamp`, chi-square gates, random generators |
| `config` | configuration as frozen dataclasses: `param(default, doc=, unit=, min=, max=, choices=, live=)`, `nested(...)`; `build()` validates and raises `ConfigError` |
| `robot` | `RobotProfile`: kinematics, base height, footprint, motion limits, frames, shared by every node through the `robot/` parameters |
| `safety` | the braking model `d = v t_r + v^2 / 2a`, the footprint's swept clearance along a translation and around a rotation, `SectorScan` semantics (blocked sectors are obstacles) |
| `io` | atomic writes with `.bak`, tolerant JSON reads, path guards against traversal, route CSV, site layout and fiducial files |
| `grid` | grid geometry, distance and likelihood fields, correlative scan-to-map matching with covariance |
| `plugins` | `Registry`: named implementations of an interface, extensible through entry points |
| `log` | structured logging (`log.info('scan', points=n)`, `throttle=`, `once=`, `event()`); with `X30_LOG_DIR`, rotated JSON lines files too |
| `ros` | `X30Node`, `run()`, topic profiles, message conversions, validated `PointCloud2`, TF, markers, diagnostics |
| `testing` | `check_package` (module size, pure `core/`, no shadowed `X30Node` members, Python 3.8), a private rosmaster, `Collector`, `wait_for`, the `ros` pytest fixture |

## X30Node

```python
from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import param
from x30_utils.ros import X30Node, run

NAME = 'greeter'


@dataclass(frozen=True)
class GreeterConfig:
    rate: float = param(1.0, min=0.1, max=50.0, unit='Hz', live=False)
    who: str = param('world', doc='who to greet')


class GreeterNode(X30Node):
    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(GreeterConfig)
        self.robot = self.declare_robot().value
        self.every(1.0 / self.config.rate, self.tick)

    def tick(self):
        self.log.info('hello', who=self.config.who, robot=self.robot.name, throttle=5.0)


def main():
    run(GreeterNode, NAME)
```

- **Parameters.** `declare_config` reads the node's private namespace (`~rate`, `~gains/kp` for
  a dotted `gains.kp`) and validates every value: a bad one stops the node at start with the
  reason (exit code 2). `rosparam set /greeter/who dharma` applies within a second (live fields
  are polled); a read-only field changed on the server is ignored with a warning until a
  restart, an invalid live value is rejected and the old one kept. Parameters no field declares
  are reported once (typos).
- **Robot profile.** `declare_robot()` reads `robot/`, searched up from the node (`~robot`,
  then `<namespace>/robot`, then `/robot`): one `robot.yaml` loaded once serves every node. It
  is read-only.
- **Callbacks one at a time.** Subscriptions, services and timers of a node never run at the
  same time (one lock), as on a single-threaded executor: no node needs locks of its own. An
  exception is logged and the node goes on; `every(..., on_error=)` hands it over instead.
- **Steady timers.** `every(period, cb, steady=True)` runs on monotonic time, immune to
  simulated time and clock jumps (watchdogs); without `steady` a timer follows ROS time.
- **Raw subscriptions.** `create_subscription(type_names, topic, cb, raw=True)` passes the
  serialized bytes (e.g. a Livox `CustomMsg` without the Livox driver installed), after
  checking the publisher's type.
- **Shutdown.** On SIGINT / SIGTERM, `rosnode kill` or a master shutdown, `run()` stops the
  timers and callbacks (a running one gets 2 s), runs the shutdown hooks while the topics are
  still open, waits until their messages are written, then unregisters. The first signal arms
  an exit deadline (8 s): a node still alive then prints every thread's stack and exits with 1.

Topic profiles: `LATCHED` (last value for late subscribers), `SENSOR` (newest only, large
buffer, no Nagle), `COMMAND` (newest only, no Nagle), `EVENTS` (queue of 20).

## Tests

`x30_add_pytests(<dir>)`, exported to every package that depends on `x30_utils`, runs pytest
under `catkin_make run_tests`. ROS tests use the `ros` fixture: a private `rosmaster` on a free
port and one rospy node for the test process.

```bash
catkin_make run_tests_x30_utils && catkin_test_results build/test_results/x30_utils
```

x30_utils is released under the [Apache License 2.0](../LICENSE), like `dharma_utils` it is
ported from.
