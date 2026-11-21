#!/usr/bin/env bash
# x30_ros across the X30's two computers: the navigation host (192.168.1.106, the intelligent
# controller: the robot's ROS master, the driver, control, LiDAR scans, odometry, localization,
# routes without a map, navigation, health, inspection, the handheld's gateway) and the
# perception host (192.168.1.105: the inspection cameras and their recorder). Sets each one's
# ROS network, checks that the two see each other, and starts each one's launch file, by hand
# or as a systemd service (x30_bringup README, "Two hosts").
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: x30_host.sh build [catkin_make args]       the workspace, as the robot runs it: Release,
                                                 no tests, on the perception host its packages
       x30_host.sh role                           this computer's role, nothing when neither
       x30_host.sh env [ROLE]                     the ROS network of this shell:
                                                 eval "$(bash tools/x30_host.sh env)"
       x30_host.sh check                          the network, the master, every node, the clocks;
                                                 Deep Robotics' autonomy (navigation host), the
                                                 cameras (perception host)
       x30_host.sh start [ROLE]                   the host's launch file (what the service runs)
       sudo bash x30_host.sh install [ROLE]       the service x30-<role>, its settings in
                                                 /etc/x30/<role>.env
ROLE: navigation (192.168.1.106) or perception (192.168.1.105); left out, this computer's by
its address. On the robot the other host's is refused: the hosts' names say otherwise
(Deep Robotics calls the navigation host ysc-perception).
EOF
  exit 2
}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
NAV_IP=${X30_NAV_IP:-192.168.1.106}
PER_IP=${X30_PER_IP:-192.168.1.105}
MASTER_PORT=11311
ETC=${X30_ETC:-/etc/x30}
SYSTEMD_DIR=${X30_SYSTEMD_DIR:-/etc/systemd/system}
SYSTEMCTL=${X30_SYSTEMCTL:-systemctl}
PROC=${X30_PROC:-/proc}
SETTLE_SEC=${X30_SETTLE_SEC:-10}
# the exit for what no restart mends, the other host's role or a settings file systemd reads
# wrong (sysexits' EX_CONFIG): the service is not restarted for it
EX_CONFIG=78
# what the perception host builds: its camera drivers, the recorder and what launches them
# (x30_bringup README, "Two hosts"); check_deps.sh --perception checks their dependencies
PERCEPTION_PACKAGES='x30_utils;x30_msgs;x30_recorder;x30_bringup'
PERCEPTION_PACKAGES+=';hikmicro_msgs;hikmicro_driver;hertzino_msgs;hertzino_driver'
# the navigation host's nodes that drive and guard the robot: the watchdog restarts the stack
# when one stays away (a respawn takes seconds)
CRITICAL=(/x30/x30_driver /control_mux_node /lidar_sector_node /path_follower_node)

say() { printf '%s\n' "$*"; }

role_ip() { # <role>
  case "$1" in
    navigation) say "$NAV_IP" ;;
    perception) say "$PER_IP" ;;
    *) usage ;;
  esac
}

workspace() { # the catkin workspace holding this checkout: <ws>/src/x30_ros/tools
  (cd "$HERE/../../.." && pwd)
}

this_role() { # navigation or perception by this computer's addresses, nothing when neither
  local ips
  ips=" $(local_ips | tr '\n' ' ') "
  case "$ips" in
    *" $NAV_IP "*) say navigation ;;
    *" $PER_IP "*) say perception ;;
  esac
}

host_role() { # [<role>]: the one asked for, else this computer's; never the other host's
  local here
  here=$(this_role)
  if [ -z "$1" ]; then
    if [ -z "$here" ]; then
      say "neither $NAV_IP nor $PER_IP is this computer's: say which, navigation or perception" >&2
      exit 2
    fi
    say "$here"
    return 0
  fi
  role_ip "$1" > /dev/null
  if [ -n "$here" ] && [ "$1" != "$here" ]; then
    say "this is the $here host ($(role_ip "$here")), the $1 host is $(role_ip "$1"):" \
      "leave the role out, it is this computer's" >&2
    exit "$EX_CONFIG"
  fi
  say "$1"
}

cmd_env() { # [<role>]
  local role
  role=$(host_role "$1")
  say "export ROS_MASTER_URI=http://$NAV_IP:$MASTER_PORT"
  say "export ROS_IP=$(role_ip "$role")"
  say "unset ROS_HOSTNAME"
  say "export PYTHONNOUSERSITE=1"
  if [ "$role" = navigation ] && [ -x "$(workspace)/neupan/bin/python" ]; then
    say "export X30_PYTHON=$(workspace)/neupan/bin/python"
  fi
}

local_ips() { # this computer's addresses (X30_LOCAL_IPS: a test's), without prefix lengths
  if [ -n "${X30_LOCAL_IPS+set}" ]; then
    say "$X30_LOCAL_IPS"
    return 0
  fi
  hostname -I 2>/dev/null || ip -o -4 addr show 2>/dev/null | awk '{sub("/.*", "", $4); print $4}' ||
    true
}

row() { printf '%-8s %-5s %s\n' "$1" "$2" "$3"; }

dr_row() { # Deep Robotics' autonomy on the navigation host: off, or what of it runs
  local running
  [ -d "${JY_COG:-$HOME/jy_cog}" ] || return 0
  running=$(bash "$HERE/dr_services.sh" sweep --dry-run 2>/dev/null | grep -c '^stop ' || true)
  if [ "$running" = 0 ]; then
    row deeprob ok "its autonomy is off"
  else
    row deeprob WARN "$running processes of its autonomy run: bash tools/dr_services.sh off"
  fi
}

launch_args() { # <role>: X30_LAUNCH_ARGS here, else the service's
  if [ -n "${X30_LAUNCH_ARGS:-}" ]; then
    say "$X30_LAUNCH_ARGS"
  elif [ -r "$ETC/$1.env" ]; then
    # systemd's quotes around the value, not around one word of it
    sed -n 's/^X30_LAUNCH_ARGS=//p' "$ETC/$1.env" | tail -n 1 | sed 's/^"\(.*\)"$/\1/'
  fi
}

cmd_check() {
  local ips role other problems=0 pid unit='' cameras=() commented
  ips=" $(local_ips | tr '\n' ' ') "
  role=$(this_role)
  if [ -n "$role" ]; then
    other=$PER_IP
    [ "$role" = perception ] && other=$NAV_IP
    row host ok "the $role host"
    commented=$(settings_comments "$role")
    if [ -n "$commented" ]; then
      row settings MISS "$ETC/$role.env, a comment after a value: ${commented//$'\n'/; }"
      problems=1
    fi
  else
    row host WARN "neither $NAV_IP nor $PER_IP here: a laptop? (ROS_IP: one of this computer's)"
    other=$NAV_IP
  fi
  if [ -z "${ROS_IP:-}" ]; then
    row ros_ip MISS "ROS_IP unset: eval \"\$(bash tools/x30_host.sh env ${role:-perception})\""
    problems=1
  elif [[ "$ROS_IP" == 127.* ]]; then
    row ros_ip MISS "ROS_IP $ROS_IP: loopback, the other host cannot reach this one's nodes"
    problems=1
  elif [[ "$ips" != *" $ROS_IP "* ]]; then
    row ros_ip MISS "ROS_IP $ROS_IP is not an address of this computer"
    problems=1
  else
    row ros_ip ok "$ROS_IP"
  fi
  if ping -c 1 -W 1 "$other" > /dev/null 2>&1; then
    row network ok "$other answers"
  else
    row network MISS "$other does not answer ping"
    problems=1
  fi
  if command -v timedatectl > /dev/null; then
    case "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" in
      yes) row ntp ok "synchronized" ;;
      *) row ntp WARN "not synchronized: the clock row says how far the hosts are apart" ;;
    esac
  fi
  if [ "$role" = navigation ]; then
    dr_row
    pid=$(master_pid)
    [ -z "$pid" ] || unit=$(unit_of "$pid")
  elif [ "$role" = perception ]; then
    cameras=(--cameras --launch-args "$(launch_args perception)")
  fi
  python3 "$HERE/x30_check.py" --master "${ROS_MASTER_URI:-http://$NAV_IP:$MASTER_PORT}" \
    --master-unit "$unit" --other "$other" "${cameras[@]}" || problems=1
  return "$problems"
}

settle_dr() { # stop Deep Robotics' autonomy until a round, 30 s after the first, finds none
  local round out
  for round in 1 2 3 4 5 6; do
    out=$(bash "$HERE/dr_services.sh" off 2>&1) || true
    grep -E '^  (stop|left) ' <<< "$out" || true
    if [ "$round" -ge 4 ] && ! grep -q '^  stop ' <<< "$out"; then
      return 0
    fi
    [ "$round" -lt 6 ] && sleep "$SETTLE_SEC"
  done
}

settings_comments() { # <role>: the variables of its settings file with a comment after them
  # systemd reads no comment after a value: it stays in the value (a token, a password, the
  # launch arguments); their lines and names only, never what a value holds
  [ -r "$ETC/$1.env" ] || return 0
  grep -nE '^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*=.*[[:space:]]#' "$ETC/$1.env" |
    sed -E 's/^([0-9]+):[[:space:]]*([A-Za-z_][A-Za-z0-9_]*)=.*/line \1, \2/' || true
}

ros_setup() { # <setup.bash>: ROS's and the workspace's, sourced as on the robot
  # their hooks read variables a service's environment has not set (ROS_DISTRO) and end on
  # failed tests: never under set -eu, or the service dies before its launch
  set +eu
  # shellcheck disable=SC1090
  source "$1"
  set -eu
}

cmd_build() { # [catkin_make arguments]: the workspace, as the robot runs it
  local ws ros top toplevel role jobs mem only=''
  ws=${X30_WS:-$(workspace)}
  ros=${X30_ROS_SETUP:-/opt/ros/noetic/setup.bash}
  toplevel=$(dirname "$ros")/share/catkin/cmake/toplevel.cmake
  top=$ws/src/CMakeLists.txt
  # the workspace's own CMakeLists.txt is a link to catkin's (catkin_make makes it); another
  # one (copied along, left by an older workspace) fails the configuration
  if [ -f "$toplevel" ] && { [ -L "$top" ] || [ -e "$top" ]; } && ! cmp -s "$top" "$toplevel"; then
    mv -f -- "$top" "$top.orig"
    ln -s "$toplevel" "$top"
    say "$top was not catkin's: kept as CMakeLists.txt.orig, catkin's linked"
  fi
  # Deep Robotics' catkin on the perception host includes the workspace's src/config.cmake in
  # its toplevel.cmake (their workspaces carry one): an empty one, or the configuration fails
  # (include could not find load file: config.cmake)
  if grep -qE '^[[:space:]]*include\([[:space:]]*config\.cmake[[:space:]]*\)' "$toplevel" \
    2> /dev/null && [ ! -e "$ws/src/config.cmake" ]; then
    say "# empty: this host's catkin (its toplevel.cmake) includes it (x30_host.sh build)" \
      > "$ws/src/config.cmake"
    say "$ws/src/config.cmake: empty, this host's catkin includes it"
  fi
  role=$(this_role)
  # the perception host builds its own packages only (neither FAST-LIO on its ARM cores nor
  # what the navigation needs); anywhere else the whole workspace, a list once cached cleared
  [ "$role" = perception ] && only=$PERCEPTION_PACKAGES
  # FAST-LIO's compile units take up to 2 GB each: no more at once than the memory holds
  jobs=$(nproc 2> /dev/null || echo 2)
  mem=$(awk '/^MemTotal:/ {print int($2 / 2097152)}' "$PROC/meminfo" 2> /dev/null || true)
  if [ -n "$mem" ] && [ "$mem" -lt "$jobs" ]; then
    jobs=$((mem > 0 ? mem : 1))
  fi
  ros_setup "$ros"
  # a newer setuptools in ~/.local (pip install --user) breaks catkin's reading of setup.py
  export PYTHONNOUSERSITE=1
  say "x30_ros: building $ws${role:+ (the $role host)}, $jobs jobs at once"
  catkin_make -C "$ws" -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    "-DCATKIN_WHITELIST_PACKAGES=$only" -j"$jobs" "$@" || return
  if [ -z "$role" ]; then
    return 0
  elif [ -f "$SYSTEMD_DIR/x30-$role.service" ]; then
    say "built; the service runs it from its next start:"
    say "  sudo systemctl restart x30-$role"
  else
    say "built; next, the service:"
    say "  sudo bash $HERE/x30_host.sh install"
  fi
}

cmd_start() { # [<role>]
  local role ws args=() setup sweep commented
  role=$(host_role "$1")
  commented=$(settings_comments "$role")
  if [ -n "$commented" ]; then
    say "$ETC/$role.env: a comment after a value becomes part of it (systemd reads none" \
      "there), put it on a line of its own: ${commented//$'\n'/; }" >&2
    exit "$EX_CONFIG"
  fi
  ws=${X30_WS:-$(workspace)}
  ros_setup "${X30_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
  setup=$ws/devel/setup.bash
  [ -f "$setup" ] || setup=$ws/install/setup.bash
  if [ ! -f "$setup" ]; then
    say "no $ws/devel/setup.bash: the workspace is not built (bash $HERE/x30_host.sh build)" >&2
    exit 1
  fi
  ros_setup "$setup"
  eval "$(cmd_env "$role" | grep -v -e ROS_MASTER_URI -e ROS_IP -e X30_PYTHON)"
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://$NAV_IP:$MASTER_PORT}
  export ROS_IP=${ROS_IP:-$(role_ip "$role")}
  if [ "$role" = navigation ] && [ -z "${X30_PYTHON:-}" ] && [ -x "$ws/neupan/bin/python" ]; then
    export X30_PYTHON=$ws/neupan/bin/python
  fi
  if [ -n "${X30_PYTHON:-}" ] && [ ! -x "$X30_PYTHON" ]; then
    # the follower's launch-prefix: a missing one would keep the follower from starting at all
    say "no $X30_PYTHON: the path follower runs with the system's python (no NeuPAN)"
    unset X30_PYTHON
  fi
  # shell words: lidar_pose:='0.3 0 0.1 0 0 0' is one argument
  mapfile -t args < <(python3 -c 'import shlex, sys; print(*shlex.split(sys.argv[1]), sep="\n")' \
    "${X30_LAUNCH_ARGS:-}")
  say "x30_ros, $role host: master $ROS_MASTER_URI, ROS_IP $ROS_IP, ${args[*]:-no arguments}"
  # 24/7: ROS's logs within a budget at every start; each node's JSON lines log, rotated
  python3 "$HERE/x30_logs.py" --budget-mb "${X30_ROS_LOG_MB:-2048}" || true
  export X30_LOG_DIR=${X30_LOG_DIR-$HOME/x30/logs}
  # the master is Deep Robotics' roscore on the navigation host: wait for it, never start one
  # with the launch (it would go with every restart, and Deep Robotics' LiDAR and IMU drivers,
  # registered with it, with it); its restart restarts the stack on both hosts
  if [ "$role" = navigation ]; then
    settle_dr
    # Deep Robotics' autonomy started again later (by a service of its own) goes again
    printf -v sweep 'bash %q sweep' "$HERE/dr_services.sh"
    exec python3 "$HERE/x30_watchdog.py" "${CRITICAL[@]/#/--critical=}" --sweep="$sweep" -- \
      roslaunch --wait x30_bringup navigation_host.launch "${args[@]}"
  fi
  exec python3 "$HERE/x30_watchdog.py" -- \
    roslaunch --wait x30_bringup perception_host.launch "${args[@]}"
}

env_file() { # <role> <user> <ws>
  local role=$1 ws=$3
  say "# x30_ros on the $role host, read by x30-$role.service (tools/x30_host.sh)."
  say "X30_WS=$ws"
  say "# arguments of x30_bringup ${role}_host.launch, in quotes, e.g."
  if [ "$role" = navigation ]; then
    say "# X30_LAUNCH_ARGS=\"site:=gardu controller:=neupan\""
    say "#   lidar_frame:=livox_frame lidar_pose:='0.3 0 0.1 0 0 0'\""
  else
    say "# X30_LAUNCH_ARGS=\"hertzino:=false\""
  fi
  say "X30_LAUNCH_ARGS=\"\""
  say "ROS_MASTER_URI=http://$NAV_IP:$MASTER_PORT"
  say "ROS_IP=$(role_ip "$role")"
  say "# logs: each node's JSON lines (default ~/x30/logs, rotated; empty: none), ROS's own"
  say "# within a budget at every start (MB)"
  say "# X30_LOG_DIR=/home/$2/x30/logs"
  say "# X30_ROS_LOG_MB=2048"
  if [ "$role" = navigation ]; then
    say "# the NeuPAN venv (tools/install_neupan.sh), for controller:=neupan"
    if [ -x "$ws/neupan/bin/python" ]; then
      say "X30_PYTHON=$ws/neupan/bin/python"
    else
      say "# X30_PYTHON=$ws/neupan/bin/python"
    fi
    say "# the handheld's shared secret (x30_bridge, the same in the app): this file is readable"
    say "# by root and $2 only; empty: anyone on the robot's Wi-Fi can drive"
    say "X30_TELEOP_TOKEN="
    say "# NETRA's robot token (x30_fleet; netra_url:=... netra_robot_id:=... in X30_LAUNCH_ARGS"
    say "# turn the uplink on): this file is readable by root and $2 only"
    say "X30_NETRA_TOKEN="
  else
    say "# the cameras' passwords: this file is readable by root and $2 only"
    say "HIKMICRO_PASSWORD="
    say "HERTZINO_PASSWORD="
  fi
}

master_pid() { # the process listening on the master's port here, '' when none (or not seen)
  command -v ss > /dev/null || return 0
  ss -H -ltnp "sport = :$MASTER_PORT" 2>/dev/null | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -n 1
}

unit_of() { # <pid>: the systemd service it runs in, '' when none
  sed -n 's#.*/\([^/]*\.service\)$#\1#p' "$PROC/$1/cgroup" 2>/dev/null | head -n 1
}

dr_units() { # Deep Robotics' services here and the ROS master's, sorted, on one line
  local pid
  {
    "$SYSTEMCTL" list-unit-files --no-legend 'jy_*.service' 2>/dev/null | awk '{print $1}'
    pid=$(master_pid)
    [ -z "$pid" ] || unit_of "$pid"
  } | awk '/^[^ ]+[.]service$/' | sort -u | tr '\n' ' ' | sed 's/ $//'
}

unit_file() { # <role> <user> [<units to start after>]
  local role=$1 after=${3:+ $3}
  cat <<EOF
[Unit]
Description=x30_ros, $role host (x30_bringup ${role}_host.launch)
Wants=network-online.target
After=network-online.target$after
# a robot's stack is never given up on: restarted however often it fails
StartLimitIntervalSec=0

[Service]
User=$2
EnvironmentFile=$ETC/$role.env
ExecStart=/bin/bash $HERE/x30_host.sh start $role
# x30_watchdog.py: READY once the master and the critical nodes answer, then a ping every 5 s
Type=notify
NotifyAccess=main
WatchdogSec=60
TimeoutStartSec=360
Restart=on-failure
# the other host's role or a settings file read wrong (x30_host.sh exits $EX_CONFIG): no
# restart helps that
RestartPreventExitStatus=$EX_CONFIG
RestartSec=10
KillSignal=SIGINT
TimeoutStopSec=70

[Install]
WantedBy=multi-user.target
EOF
}

remove_misplaced() { # <role>: the other host's service and settings, installed here by mistake
  local other=perception
  [ "$1" = perception ] && other=navigation
  # only where this computer is one of the hosts (a test machine may hold both)
  if [ -z "$(this_role)" ] || [ ! -f "$SYSTEMD_DIR/x30-$other.service" ]; then
    return 0
  fi
  "$SYSTEMCTL" disable --now "x30-$other.service" > /dev/null 2>&1 || true
  rm -f -- "$SYSTEMD_DIR/x30-$other.service" "$ETC/$other.env"
  say "removed x30-$other.service and $ETC/$other.env: the $other host is $(role_ip "$other")"
}

cmd_install() { # [<role>]
  local role user ws env unit after='' what
  role=$(host_role "$1")
  if [ "$(id -u)" != 0 ] && [ -z "${X30_ETC:-}" ]; then
    say "run it with sudo: sudo bash $HERE/x30_host.sh install${1:+ $1}"
    return 1
  fi
  user=${SUDO_USER:-$(id -un)}
  ws=$(workspace)
  env=$ETC/$role.env
  unit=$SYSTEMD_DIR/x30-$role.service
  mkdir -p "$ETC"
  if [ -f "$env" ]; then
    say "kept $env (its settings stay)"
  else
    env_file "$role" "$user" "$ws" > "$env"
    say "wrote $env"
  fi
  chmod 600 "$env"
  chown "$user" "$env" 2>/dev/null || true
  if [ "$role" = navigation ]; then
    # after Deep Robotics' own services, the master's among them: what they start is settled
    # (dr_services.sh off) once they are up
    after=$(dr_units)
    if [ -n "$after" ]; then
      say "starts after $after"
    else
      after='jy_roscore.service jy_core.service jy_cog.service'
      say "no Deep Robotics service found (the master not running?): after $after by name"
    fi
  fi
  remove_misplaced "$role"
  unit_file "$role" "$user" "$after" > "$unit"
  say "wrote $unit"
  "$SYSTEMCTL" daemon-reload
  "$SYSTEMCTL" enable "x30-$role.service"
  [ "$role" = perception ] && what='X30_LAUNCH_ARGS, the passwords' ||
    what="X30_LAUNCH_ARGS, the handheld's X30_TELEOP_TOKEN, NETRA's X30_NETRA_TOKEN"
  # one command a line: copied from a narrow screen, nothing runs half
  say "next, the settings ($what):"
  say "  sudo nano $env"
  say "then the service, and its log:"
  say "  sudo systemctl restart x30-$role"
  say "  journalctl -u x30-$role -f"
}

main() {
  [ $# -ge 1 ] || usage
  case "$1" in
    build) shift; cmd_build "$@" ;;
    role) [ $# = 1 ] || usage; this_role ;;
    env) [ $# -le 2 ] || usage; cmd_env "${2:-}" ;;
    check) [ $# = 1 ] || usage; cmd_check ;;
    start) [ $# -le 2 ] || usage; cmd_start "${2:-}" ;;
    install) [ $# -le 2 ] || usage; cmd_install "${2:-}" ;;
    *) usage ;;
  esac
}

main "$@"
