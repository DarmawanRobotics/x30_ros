# tools

Bash scripts for the X30 itself and for getting the stack onto it: no ROS, nothing to build,
nothing downloaded on the robot.

| Script | Run on | |
|---|---|---|
| `dr_services.sh status\|off\|on` | the navigation host (192.168.1.106) | Deep Robotics' own localization, navigation, mapping, URDF and bridge off, now and at boot, and back on |
| `check_deps.sh [--perception] [--tests] [--gui]` | the robot | what the build and the stack need, checked offline: the apt packages, the sources of x30.repos, the Livox SDK; on the perception host (by its address, or `--perception`) what it builds (the camera drivers, the recorder) |
| `install_neupan.sh [--venv DIR] [--wheels DIR]`, `--download DIR`, `--check` | the navigation host | NeuPAN for the `neupan` controller: a venv with PyTorch (CPU), cvxpy and NeuPAN with the patches of [neupan/](neupan) |
| `x30_host.sh build\|role\|env\|check\|start\|install` | each host | the workspace built as the robot needs it, the ROS network of the navigation (192.168.1.106) and the perception host (192.168.1.105), whether they see each other, and each one's launch file by hand or as the service `x30-<role>`; the role is the host's own, by its address |
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
4. The controller app's navigation recorder needs Deep Robotics' navigation; x30_navigation's
   routes replace it. Charging (`charge_start`) belongs to the perception host and stays.

**Which IMU.** On the navigation host `/imu/data` comes from `/yesense_imu_node`. If that is the
intelligent controller's own IMU (Application Manual 6.3: (87.15, −1, 117) mm in the body frame
at mounting position A, (47.15, −1, 117) at B, (7.15, −1, 117) at C) rather than the body's
(130.45, −1, 14.9), FAST-LIO's `extrinsic_T` in
[x30_odometry/config/fast_lio.yaml](../x30_odometry/config/fast_lio.yaml) is minus that position
(the axes are the body's in both cases). Check it before the first map:
`rosnode info /yesense_imu_node`, and the frame of `rostopic echo -n 1 /imu/data/header`.

## Two hosts and the watchdog

`x30_host.sh` (x30_bringup README, "Two hosts", has the split and the why). ROLE is
`navigation` or `perception`; left out, this computer's by its address, and on the robot the
other host's is refused (exit 78, which its service is never restarted for): Deep Robotics
names the navigation host `ysc-perception`.

| | |
|---|---|
| `build [ARGS]` | catkin_make as the robot needs it: ROS sourced, `PYTHONNOUSERSITE=1`, `-DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release`, one job per 2 GB of memory at most, on the perception host its packages only (`CATKIN_WHITELIST_PACKAGES`, cleared anywhere else); a workspace `CMakeLists.txt` that is not catkin's replaced (kept as `CMakeLists.txt.orig`); ARGS go to catkin_make |
| `role` | this computer's role, nothing when neither |
| `env [ROLE]` | the exports of that host: `ROS_MASTER_URI=http://192.168.1.106:11311`, its `ROS_IP`, no `ROS_HOSTNAME`, `PYTHONNOUSERSITE=1`, and `X30_PYTHON` when NeuPAN's venv is there; `eval` it |
| `check` | this host's role, `ROS_IP` one of its addresses, the other host answering ping, NTP, the master answering (on the navigation host with the systemd service running it), every registered node reachable from here (a name that does not resolve: `/etc/hosts`), the other host's clock against this one's (below); on the navigation host Deep Robotics' autonomy off, on the perception host the cameras' ports (HIKMICRO 80 and 554, Hertzino 80 and 443, at the addresses of `X30_LAUNCH_ARGS` or `/etc/x30/perception.env`) |
| `start [ROLE]` | what the service runs: ROS and the workspace sourced (outside `set -eu`: ROS's hooks read unset variables), the network set (the service's settings win), ROS's logs pruned to `X30_ROS_LOG_MB` (2048, `x30_logs.py`: the oldest runs first, the newest 5 kept), each node's JSON lines log in `X30_LOG_DIR` (`~/x30/logs`; empty: none), on the navigation host `dr_services.sh off` until Deep Robotics' boot settled, then the host's launch file with `X30_LAUNCH_ARGS` under `x30_watchdog.py`, waiting for Deep Robotics' master (never one of its own: it would go with every restart, and Deep Robotics' LiDAR and IMU drivers with it); on the navigation host the watchdog runs `dr_services.sh sweep` every 30 s, so what one of Deep Robotics' services starts again late goes again (the journal names it) |
| `install [ROLE]` | with sudo: `/etc/x30/<role>.env` (kept when it exists; mode 600, the cameras' passwords, the handheld's token and NETRA's go there) and `/etc/systemd/system/x30-<role>.service` (`Type=notify`, `WatchdogSec=60`, `Restart=on-failure`, never given up on: `StartLimitIntervalSec=0`, but not on the wrong host: `RestartPreventExitStatus=78`), enabled; the other host's service and settings, put on this one by an older x30_host.sh, removed; the navigation host's starts after Deep Robotics' services (`jy_*`) and the one running the ROS master, as found (run it with the robot up) |

`X30_NAV_IP` and `X30_PER_IP` change the two addresses. The service's settings:

```bash
# /etc/x30/navigation.env (shell words: a quoted value stays one argument)
X30_LAUNCH_ARGS="site:=gardu controller:=neupan lidar_pose:='0.3 0 0.1 0 0 0' netra_url:=https://netra.example.com netra_robot_id:=x30-01"
X30_PYTHON=/home/ysc/x30_ws/neupan/bin/python
X30_TELEOP_TOKEN=...            # the handheld's shared secret (x30_bridge, the same in the app)
X30_NETRA_TOKEN=...             # NETRA's robot token (x30_fleet; NETRA: Robots, the robot's token)
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
`--sweep CMD [--sweep-sec 30]` runs a command beside the stack and logs what it says (the same
output once).

The clock row takes the stamps of a small topic the other host publishes (`/x30/odom` from the
navigation host, the cameras' state from the perception host) against this host's clock when
they arrive: `in step` within 0.1 s, else how far apart. Nothing in x30_ros compares one host's
stamps with the other's clock (the inspection calls the camera drivers' services), so a skew
breaks no node; logs, reports and bags of the two hosts disagree by it. Without internet the
navigation host can be the perception host's clock: `chrony` serving the robot's network on
the navigation host (`allow 192.168.1.0/24`, `local stratum 10`) and following it on the
perception host (`server 192.168.1.106 iburst`), or once after boot
`sudo date -s "$(ssh ysc@192.168.1.106 date -Ins)"` on the perception host.

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

How fast NeuPAN solves on the navigation host, before the robot walks with it (a few seconds,
the robot standing still): `$X30_PYTHON $(rospack find x30_navigation)/scripts/bench_neupan`
(x30_navigation README, "NeuPAN").

## Robot without internet

1. On a computer with internet (Linux or macOS, with git):

   ```bash
   git clone https://github.com/DarmawanRobotics/x30_ros.git && cd x30_ros
   bash tools/pack_sources.sh x30_sources.tar.gz
   scp x30_sources.tar.gz ysc@192.168.1.106:~/
   scp x30_sources.tar.gz ysc@192.168.1.105:~/
   ```

   `--skip drivers/hikmicro_driver` leaves out a repository the robot has already.

   The archive holds `src/` (x30_ros as checked out, FAST_LIO with its ikd-Tree,
   livox_ros_driver, the camera drivers; `src/SOURCES.txt` lists their commits) and
   `Livox-SDK/`, without git history or FAST_LIO's videos.

2. On each host of the robot, a clean workspace (the one before kept as `src.prev`: a package
   x30_ros no longer has would stay in a tree unpacked over the old one), then its build:

   ```bash
   mkdir -p ~/x30_ws && cd ~/x30_ws
   rm -rf src.prev build devel
   [ -d src ] && mv src src.prev
   tar -xzf ~/x30_sources.tar.gz
   bash src/x30_ros/tools/check_deps.sh
   bash src/x30_ros/tools/x30_host.sh build
   ```

   Once, on the navigation host, the Livox SDK first (FAST-LIO's driver links it):

   ```bash
   cd ~/x30_ws/Livox-SDK/build
   cmake .. && make -j4
   sudo make install
   ```

   `check_deps.sh` reads only what is installed (`dpkg`, the workspace, `/usr/local/lib`); it
   lists every missing package with one `apt install` line, a source package missing or there
   twice (a copy of hikmicro_driver next to the archive's: catkin builds one), and the Livox
   SDK. Where the robot has a rosdep cache, `rosdep check` runs too. `--tests` adds what
   `catkin_make run_tests` needs, `--gui` RViz. A setuptools that `pip install --user` put in
   `~/.local` stops catkin (`module 'importlib_metadata' has no attribute 'EntryPoints'`):
   `check_deps.sh` warns, and `x30_host.sh build` sets `PYTHONNOUSERSITE=1`, which leaves
   `~/.local` out without touching it. On the perception host both go by its address: its
   dependencies, its packages.

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
