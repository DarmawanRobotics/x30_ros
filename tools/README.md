# tools

Bash scripts for the X30 itself and for getting the stack onto it: no ROS, nothing to build,
nothing downloaded on the robot.

| Script | Run on | |
|---|---|---|
| `dr_services.sh status\|off\|on` | the navigation host (192.168.1.106) | Deep Robotics' own localization, navigation, mapping, URDF and bridge off, now and at boot, and back on |
| `check_deps.sh [--tests] [--gui]` | the robot | what the build and the stack need, checked offline: the apt packages, the sources of x30.repos, the Livox SDK |
| `install_neupan.sh [--venv DIR] [--wheels DIR]`, `--download DIR`, `--check` | the navigation host | NeuPAN for the `neupan` controller: a venv with PyTorch (CPU), cvxpy and NeuPAN with the patches of [neupan/](neupan) |
| `x30_host.sh env\|check\|start\|install` | each host | the ROS network of the navigation (192.168.1.106) and the perception host (192.168.1.105), whether they see each other, and each one's launch file by hand or as the service `x30-<role>` |
| `x30_watchdog.py` | (the service) | the host's roslaunch under systemd's watchdog: restarts it when the master restarts or a critical node stays away |
| `pack_sources.sh [OUT] [--skip PATH]` | a computer with internet | x30_ros, the repositories of x30.repos and the Livox SDK in one archive (about 5 MB) |

## Deep Robotics' own autonomy

The navigation host (the intelligent controller on the robot's back, 192.168.1.106) runs Deep
Robotics' localization, navigation and mapping from `~/jy_cog` (Application Manual V2.0.4,
chapter 3). Next to x30_ros they only take CPU, and they get in its way: both localizations
take `/initialpose`, Deep Robotics' navigation publishes a `/map` of its own and its odometry's
TF, and its `/cmd_vel` reaches the motion host through the perception host's elevation map in
navigation mode.

```bash
bash tools/dr_services.sh status   # autorun_slam, what runs, the busiest, port 43897, boot starters
bash tools/dr_services.sh off      # stop it now, and not again at boot
bash tools/dr_services.sh on       # autorun_slam back as it was (then reboot, or start_nav.sh)
ssh ysc@192.168.1.106 'bash -s -- off' < tools/dr_services.sh   # the same from a laptop
```

`off` sets `~/jy_cog/system/conf/autorun_slam` to `false` (the original kept as
`autorun_slam.x30_backup`; read at the next boot of the navigation host, manual 3.2), runs Deep
Robotics' own `kill_nav.sh` and `kill_mapping.sh` (`stop_record.sh` while a recording runs), and
stops what of jy_cog still runs of localization, navigation (`move_base` and the local planner:
`plan.sh`, `local_planner.sh`, `pcl_concatenate`, `localPlanner`), mapping, the player, the
recorder, RViz, Deep Robotics' URDF publisher (`drivers/scripts/urdf.sh`: x30_description
publishes those frames, and a frame with two publishers jumps between them) and its bridge to
the robot (`transfer`: `udp_receiver` holds UDP 43897, where the motion host's reports arrive
for x30_driver, and `udp_sender` drives the robot from `/cmd_vel`, past the control mux;
x30_driver does both now). All of them at once: SIGINT, then TERM, then KILL, so no wrapper
script restarts its node.

A process is judged by its program inside jy_cog (the binary, the script a shell runs, the
launch file), never by its arguments: `localization.sh false livox` is the localization. A
process with no program there (the `robot_state_publisher` that `urdf.sh` runs, a roslaunch's
nodes) goes with its nearest ancestor that has one, also once that ancestor is gone; a script
of the autonomy that also runs something kept is left (`left ...`), since stopping it could
take the kept one along. `--dry-run` prints all of it and changes nothing; `on` restores
`autorun_slam` from the backup.

Kept: the navigation host's sensor drivers (`/livox_lidar_publisher2`, `/yesense_imu_node`:
`/livox/lidar`, `/lidar_points`, `/imu/data`), its ROS master and the robot server
(`robot_server`, formerly `broker`: the controller app). Listed under "other" and left running:
`cpu_monitor`. Never touched: the motion host (192.168.1.103: the walking) and the perception
host (192.168.1.105: the elevation map, autonomous charging on UDP 3333 and their drivers);
x30_driver uses both, and the script refuses to run there.

`status` also tells who holds UDP 43897 (`free` once the bridge is off; another process: give
the driver another `local_port` and list that port in the motion host's `network.toml`) and
which files start jy_cog at boot (`file:` lines).

After `off`:

1. `rostopic hz /livox/lidar /imu/data`: still 10 Hz and 200 Hz.
2. `rostopic info /map` and `rostopic info /initialpose`: no Deep Robotics node left among the
   publishers and subscribers.
3. `dr_services.sh status` again: nothing under "Deep Robotics autonomy, URDF and bridge", and
   `udp 43897 (x30_driver's reports): free`. A heavy process still listed under "other jy_cog
   processes" is left alone on purpose: look at it first.
4. The controller app's navigation recorder needs Deep Robotics' navigation; x30_mapping's
   routes replace it. Charging (`charge_start`) belongs to the perception host and stays.

**Which IMU.** On the navigation host `/imu/data` comes from `/yesense_imu_node`. If that is the
intelligent controller's own IMU (Application Manual 6.3: (87.15, −1, 117) mm in the body frame
at mounting position A, (47.15, −1, 117) at B, (7.15, −1, 117) at C) rather than the body's
(130.45, −1, 14.9), FAST-LIO's `extrinsic_T` in
[x30_odometry/config/fast_lio.yaml](../x30_odometry/config/fast_lio.yaml) is minus that position
(the axes are the body's in both cases). Check it before the first map:
`rosnode info /yesense_imu_node`, and the frame of `rostopic echo -n 1 /imu/data/header`.

## Two hosts and the watchdog

`x30_host.sh` (x30_bringup README, "Two hosts", has the split and the why):

| | |
|---|---|
| `env navigation\|perception` | the exports of that host: `ROS_MASTER_URI=http://192.168.1.106:11311`, its `ROS_IP`, no `ROS_HOSTNAME`, `PYTHONNOUSERSITE=1`, and `X30_PYTHON` when NeuPAN's venv is there; `eval` it |
| `check` | this host's role, `ROS_IP` one of its addresses, the other host answering ping, the clock synchronized, the master answering, every registered node reachable from here (a name that does not resolve: `/etc/hosts`) |
| `start navigation\|perception` | what the service runs: ROS and the workspace sourced, the network set (the service's settings win), on the navigation host `dr_services.sh off` until Deep Robotics' boot settled, then the host's launch file with `X30_LAUNCH_ARGS` under `x30_watchdog.py` |
| `install navigation\|perception` | with sudo: `/etc/x30/<role>.env` (kept when it exists; mode 600, the cameras' passwords go there) and `/etc/systemd/system/x30-<role>.service` (`Type=notify`, `WatchdogSec=60`, `Restart=on-failure`), enabled |

`X30_NAV_IP` and `X30_PER_IP` change the two addresses. The service's settings:

```bash
# /etc/x30/navigation.env
X30_LAUNCH_ARGS="site:=gardu odometry_only:=true controller:=neupan"
X30_PYTHON=/home/ysc/x30_ws/neupan/bin/python
# /etc/x30/perception.env
X30_LAUNCH_ARGS=""
HIKMICRO_PASSWORD=...
HERTZINO_PASSWORD=...
```

`x30_watchdog.py [--critical NODE]... [--grace 45] [--startup 240] [--period 5] -- roslaunch ...`
runs the launch and tells systemd READY once the master and the critical nodes answer, then
pings `WATCHDOG=1` with a status line (`systemctl status x30-navigation`) while they do. It ends
with an error, for systemd to restart the service, when the master restarted (`/run_id`
changed), when the master or a critical node stays away longer than the grace, when the start
takes longer than `--startup`, or when roslaunch exits; a hung watchdog stops pinging and
systemd restarts it after 60 s. Stopping the service stops roslaunch with SIGINT first.

## NeuPAN

`install_neupan.sh` puts NeuPAN in a venv on the navigation host, `~/x30_ws/neupan`: the venv
sees the system's and ROS's packages, PyTorch comes for the CPU only (about 200 MB, no CUDA),
cvxpy, cvxpylayers and the solvers at versions that still run on Python 3.8, and NeuPAN is
built from its `py38` branch at a pinned commit with the patches of [neupan/](neupan) (the
holonomic kinematics). It ends with `--check`: the versions and `NeuPAN with the holonomic
kinematics: ok`. The path follower runs with the venv's python when `X30_PYTHON` points at it
(navigation.launch's `launch-prefix`), so the rest of the stack never loads PyTorch.

```bash
bash tools/install_neupan.sh                       # on the navigation host, with internet
echo 'export X30_PYTHON=~/x30_ws/neupan/bin/python' >> ~/.bashrc
bash tools/install_neupan.sh --download ~/wheels   # or: on a computer with internet ...
bash tools/install_neupan.sh --wheels ~/wheels     # ... then on the robot, no internet
```

`--download` fetches every wheel for Python 3.8 on x86_64 (any OS with pip and git) and builds
NeuPAN's own. A `~/.local` setuptools or numpy never leaks in (`PYTHONNOUSERSITE=1`, as for
catkin). Without `python3-venv` it says so (`sudo apt install python3-venv`).

## Robot without internet

1. On a computer with internet (Linux or macOS, with git):

   ```bash
   git clone https://github.com/DarmawanRobotics/x30_ros.git && cd x30_ros
   bash tools/pack_sources.sh x30_sources.tar.gz   # --skip drivers/hikmicro_driver if the robot has it
   scp x30_sources.tar.gz ysc@192.168.1.106:~/
   ```

   The archive holds `src/` (x30_ros as checked out, FAST_LIO with its ikd-Tree,
   livox_ros_driver, the camera drivers; `src/SOURCES.txt` lists their commits) and
   `Livox-SDK/`, without git history or FAST_LIO's videos.

2. On the robot, in its workspace (`~/hotdog` here):

   ```bash
   mkdir -p ~/hotdog && tar -xzf ~/x30_sources.tar.gz -C ~/hotdog
   bash ~/hotdog/src/x30_ros/tools/check_deps.sh
   cd ~/hotdog/Livox-SDK/build && cmake .. && make -j"$(nproc)" && sudo make install   # once
   cd ~/hotdog && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
   ```

   `check_deps.sh` reads only what is installed (`dpkg`, the workspace, `/usr/local/lib`); it
   lists every missing package with one `apt install` line, a source package missing or there
   twice (a copy of hikmicro_driver next to the archive's: catkin builds one), and the Livox
   SDK. Where the robot has a rosdep cache, `rosdep check` runs too. `--tests` adds what
   `catkin_make run_tests` needs, `--gui` RViz. A setuptools that `pip install --user` put in
   `~/.local` stops catkin (`module 'importlib_metadata' has no attribute 'EntryPoints'`):
   `check_deps.sh` warns, and its build line then starts with `PYTHONNOUSERSITE=1`, which leaves
   `~/.local` out without touching it.

3. Missing packages: borrow the laptop's internet through SSH (the cable is enough, no Wi-Fi):

   ```bash
   ssh -R 1080 ysc@192.168.1.106   # on the laptop: a SOCKS proxy back to it while it is open
   # in that session, on the robot
   P='-o Acquire::http::Proxy=socks5h://127.0.0.1:1080 -o Acquire::https::Proxy=socks5h://127.0.0.1:1080'
   sudo apt $P update && sudo apt $P install -y <the packages check_deps.sh printed>
   ```

   Where SSH forwarding is refused, let the robot's own apt say which files it needs (from its
   package lists, offline), fetch them on the laptop and install them:

   ```bash
   sudo apt-get install --print-uris -qq <packages> | cut -d"'" -f2 > ~/urls.txt  # robot; installs nothing
   scp ysc@192.168.1.106:~/urls.txt . && mkdir -p debs \
     && (cd debs && xargs -n1 curl -fsSLO < ../urls.txt)                      # on the laptop
   scp -r debs ysc@192.168.1.106:~/ && ssh ysc@192.168.1.106 'sudo dpkg -i ~/debs/*.deb'
   ```

   "Unable to locate package" there means the robot has no package lists: use the SSH proxy.

## Tests

`x30_bringup/test/test_tools.py` (`catkin_make run_tests_x30_bringup`): `dr_services.sh` against
a fake jy_cog with processes started from it (status, off, on, a second off, dry run, refusals),
`check_deps.sh` against a fake `dpkg-query` and workspace, its package lists against every
dependency of the stack's `package.xml` files, and `pack_sources.sh` with local repositories.
