#!/usr/bin/env bash
# x30_ros across the X30's two computers: the navigation host (192.168.1.106, the intelligent
# controller: the robot's ROS master, the driver, control, LiDAR scans, odometry, localization,
# mapping, navigation, health, inspection) and the perception host (192.168.1.105: the
# inspection cameras). Sets each one's ROS network, checks that the two see each other, and
# starts each one's launch file, by hand or as a systemd service (x30_bringup README, "Two
# hosts").
set -euo pipefail

usage() {
  cat <<'EOF'
usage: x30_host.sh env navigation|perception      the ROS network of this shell:
                                                 eval "$(bash tools/x30_host.sh env perception)"
       x30_host.sh check                          the master, the other host, every node's address
       x30_host.sh start navigation|perception    the host's launch file (what the service runs)
       sudo bash x30_host.sh install navigation|perception
                                                 the service x30-<role>, its settings in
                                                 /etc/x30/<role>.env
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
SETTLE_SEC=${X30_SETTLE_SEC:-10}
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

cmd_env() { # <role>
  local ip
  ip=$(role_ip "$1")
  say "export ROS_MASTER_URI=http://$NAV_IP:$MASTER_PORT"
  say "export ROS_IP=$ip"
  say "unset ROS_HOSTNAME"
  say "export PYTHONNOUSERSITE=1"
  if [ "$1" = navigation ] && [ -x "$(workspace)/neupan/bin/python" ]; then
    say "export X30_PYTHON=$(workspace)/neupan/bin/python"
  fi
}

local_ips() { hostname -I 2>/dev/null || ip -o -4 addr show 2>/dev/null | awk '{print $4}'; }

row() { printf '%-8s %-5s %s\n' "$1" "$2" "$3"; }

cmd_check() {
  local ips role='' other problems=0
  ips=" $(local_ips | tr '\n' ' ') "
  case "$ips" in
    *" $NAV_IP "*) role=navigation other=$PER_IP ;;
    *" $PER_IP "*) role=perception other=$NAV_IP ;;
  esac
  if [ -n "$role" ]; then
    row host ok "the $role host"
  else
    row host WARN "neither $NAV_IP nor $PER_IP here: a laptop? (ROS_IP: one of this computer's)"
    other=$NAV_IP
  fi
  if [ -z "${ROS_IP:-}" ]; then
    row ros_ip MISS "ROS_IP unset: eval \"\$(bash tools/x30_host.sh env ${role:-perception})\""
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
      yes) row clock ok "synchronized (NTP)" ;;
      *) row clock WARN "not synchronized: stamps across the hosts may disagree" ;;
    esac
  fi
  python3 - "${ROS_MASTER_URI:-http://$NAV_IP:$MASTER_PORT}" <<'EOF' || problems=1
import socket
import sys
from urllib.parse import urlparse
import xmlrpc.client

master = sys.argv[1]
socket.setdefaulttimeout(2.0)
proxy = xmlrpc.client.ServerProxy(master)
try:
    _, _, state = proxy.getSystemState('/x30_host_check')
except (OSError, xmlrpc.client.Error) as error:
    print(f'{"master":<8} {"MISS":<5} {master}: {error}')
    sys.exit(1)
print(f'{"master":<8} {"ok":<5} {master}')
nodes = sorted({node for kind in state for _, names in kind for node in names})
unreachable = 0
for node in nodes:
    code, _, uri = proxy.lookupNode('/x30_host_check', node)
    if code != 1:
        continue
    address = urlparse(uri)
    try:
        socket.create_connection((address.hostname, address.port), 2.0).close()
    except OSError as error:
        unreachable += 1
        print(f'{"node":<8} {"MISS":<5} {node} at {address.hostname}:{address.port}: {error}')
if unreachable:
    print('         a name that does not resolve: "<ip> <name>" in /etc/hosts, or ROS_IP for that')
    print('         node where it starts')
print(f'{"nodes":<8} {"ok" if not unreachable else "MISS":<5} '
      f'{len(nodes) - unreachable} of {len(nodes)} reachable from here')
sys.exit(1 if unreachable else 0)
EOF
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

cmd_start() { # <role>
  local role=$1 ws args=() setup
  role_ip "$role" > /dev/null
  ws=${X30_WS:-$(workspace)}
  # shellcheck disable=SC1090 # the ROS and workspace setup, sourced as on the robot
  source "${X30_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
  setup=$ws/devel/setup.bash
  [ -f "$setup" ] || setup=$ws/install/setup.bash
  # shellcheck disable=SC1090
  source "$setup"
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
  read -r -a args <<< "${X30_LAUNCH_ARGS:-}"
  say "x30_ros, $role host: master $ROS_MASTER_URI, ROS_IP $ROS_IP, ${args[*]:-no arguments}"
  # the master is Deep Robotics' roscore on the navigation host: wait for it, never start one
  # with the launch (it would go with every restart, and Deep Robotics' LiDAR and IMU drivers,
  # registered with it, with it); its restart restarts the stack on both hosts
  if [ "$role" = navigation ]; then
    settle_dr
    exec python3 "$HERE/x30_watchdog.py" "${CRITICAL[@]/#/--critical=}" -- \
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
    say "# X30_LAUNCH_ARGS=\"site:=gardu odometry_only:=true controller:=neupan\""
  else
    say "# X30_LAUNCH_ARGS=\"hertzino:=false\""
  fi
  say "X30_LAUNCH_ARGS=\"\""
  say "ROS_MASTER_URI=http://$NAV_IP:$MASTER_PORT"
  say "ROS_IP=$(role_ip "$role")"
  if [ "$role" = navigation ]; then
    say "# the NeuPAN venv (tools/install_neupan.sh), for controller:=neupan"
    if [ -x "$ws/neupan/bin/python" ]; then
      say "X30_PYTHON=$ws/neupan/bin/python"
    else
      say "# X30_PYTHON=$ws/neupan/bin/python"
    fi
  else
    say "# the cameras' passwords: this file is readable by root and $2 only"
    say "HIKMICRO_PASSWORD="
    say "HERTZINO_PASSWORD="
  fi
}

unit_file() { # <role> <user>
  local role=$1 after=''
  [ "$role" = navigation ] && after=' jy_roscore.service jy_core.service jy_cog.service'
  cat <<EOF
[Unit]
Description=x30_ros, $role host (x30_bringup ${role}_host.launch)
Wants=network-online.target
After=network-online.target$after

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
RestartSec=10
KillSignal=SIGINT
TimeoutStopSec=70

[Install]
WantedBy=multi-user.target
EOF
}

cmd_install() { # <role>
  local role=$1 user ws env unit
  role_ip "$role" > /dev/null
  if [ "$(id -u)" != 0 ] && [ -z "${X30_ETC:-}" ]; then
    say "run it with sudo: sudo bash $HERE/x30_host.sh install $role"
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
  unit_file "$role" "$user" > "$unit"
  say "wrote $unit"
  "$SYSTEMCTL" daemon-reload
  "$SYSTEMCTL" enable "x30-$role.service"
  say "next: the settings in $env (X30_LAUNCH_ARGS$([ "$role" = perception ] && printf ', the passwords'))"
  say "then: sudo systemctl start x30-$role; journalctl -u x30-$role -f"
}

main() {
  [ $# -ge 1 ] || usage
  case "$1" in
    env) [ $# = 2 ] || usage; cmd_env "$2" ;;
    check) [ $# = 1 ] || usage; cmd_check ;;
    start) [ $# = 2 ] || usage; cmd_start "$2" ;;
    install) [ $# = 2 ] || usage; cmd_install "$2" ;;
    *) usage ;;
  esac
}

main "$@"
