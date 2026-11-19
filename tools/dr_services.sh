#!/usr/bin/env bash
# Deep Robotics' own autonomy on the X30's navigation host (192.168.1.106, ~/jy_cog): its
# localization, navigation (move_base and the local planner) and mapping take the CPU x30_ros
# needs and clash with it (/map, /initialpose, TF, /cmd_vel), its URDF publisher sends the
# frames x30_description does, and its bridge (transfer) holds the UDP port of the motion host's
# reports and drives the robot from /cmd_vel. Turns them off and on again; offline, on the
# navigation host. While x30_ros runs as a service its watchdog sweeps them every 30 s: what one
# of Deep Robotics' services starts again late goes again.
# Kept: the sensor drivers (/livox/lidar, /imu/data), the ROS master and the robot server
# (the controller app). Never touched: the motion host (.103) and the perception host (.105:
# elevation map, autonomous charging and their drivers).
set -u

usage() {
  cat <<'EOF'
usage: dr_services.sh status|off|on|sweep [--dry-run] [--force]
  status   autorun_slam, the Deep Robotics processes, the busiest, the report port, boot starters
  off      stop the localization, navigation, mapping, URDF and bridge now; autorun_slam false
  on       autorun_slam as it was before off (back from the next boot)
  sweep    stop what of them runs again, one line each, nothing else (x30_ros running: no
           Deep Robotics script, which may stop processes by name)
  --dry-run  print what would be done, change nothing
from a laptop: ssh ysc@192.168.1.106 'bash -s -- off' < tools/dr_services.sh
EOF
  exit 2
}

home_of_user() {  # the invoking user's home, also under sudo
  local user=${SUDO_USER:-${USER:-$(id -un)}} home
  home=$(getent passwd "$user" | cut -d: -f6)
  printf '%s' "${home:-$HOME}"
}

JY=${JY_COG:-$(home_of_user)/jy_cog}
CONF=$JY/system/conf/autorun_slam
BACKUP=$CONF.x30_backup
SCRIPTS=$JY/system/scripts
RECORDER=$JY/rosbag_recorder/scripts
# processes started from jy_cog, judged by their program inside it (the binary, the script a
# shell runs, the launch file), never by its arguments; lower case, KEEP wins over STOP, the
# rest is only listed. A process with no program there (the robot_state_publisher a wrapper
# script runs, a roslaunch's nodes) belongs to its nearest ancestor that has one.
STOP='^(localization|nav|planner|mapping|slam|transfer)/|locali[sz]ation|location|navigation|move_base|planner|slam|mapping|player|record|rviz|pcd2map|occ_|(^|/)urdf|message_transformer'
KEEP='livox|lidar|yesense|(^|[/_])imu|robot_server|broker|rosmaster|roscore|rosout'
REPORT_PORT=43897  # x30_driver's local_port, where the motion host's reports arrive
DRY=0
FORCE=0
FAILED=0

say() { printf '%s\n' "$*"; }

run() {
  if [ "$DRY" = 1 ]; then
    say "  would run: $*"
  else
    "$@"
  fi
}

run_quiet() {  # run, the command's own output dropped
  if [ "$DRY" = 1 ]; then
    say "  would run: $*"
  else
    "$@" > /dev/null 2>&1
  fi
}

autorun() {
  if [ -f "$CONF" ]; then tr -d ' \t\r\n' < "$CONF"; else printf 'missing'; fi
}

alive() {  # <pid>: running, and not a zombie waiting for its parent
  local key value
  while read -r key value _; do
    if [ "$key" = State: ]; then
      [ "$value" != Z ] && [ "$value" != X ]
      return
    fi
  done 2>/dev/null < "/proc/$1/status"
  return 1
}

remaining() {  # <pid>...: those still alive, once each
  local pid out=''
  for pid in "$@"; do
    alive "$pid" || continue
    case " $out " in *" $pid "*) ;; *) out="$out $pid" ;; esac
  done
  printf '%s' "${out# }"
}

program() {  # <args>: PROGRAM = the first word inside jy_cog, relative to it, lower case
  local word words
  PROGRAM=''
  read -r -a words <<< "$1"
  for word in "${words[@]}"; do
    case "$word" in
      "$JY/"*)
        PROGRAM=${word#"$JY/"}
        PROGRAM=${PROGRAM,,}
        return 0
        ;;
    esac
  done
}

declare -A PARENT=() OWN=() CPU=() ARGS=()

snapshot() {  # every process now: parent, cpu, args, own class ('' when not from jy_cog)
  local pid ppid cpu args
  PARENT=() OWN=() CPU=() ARGS=()
  while read -r pid ppid cpu args; do
    [ "$pid" = "$$" ] && continue
    PARENT[$pid]=$ppid
    CPU[$pid]=$cpu
    ARGS[$pid]=$args
    program "$args"
    if [ -z "$PROGRAM" ]; then
      OWN[$pid]=''
    elif [[ $PROGRAM =~ $KEEP ]]; then
      OWN[$pid]=keep
    elif [[ $PROGRAM =~ $STOP ]]; then
      OWN[$pid]=stop
    else
      OWN[$pid]=other
    fi
  done < <(ps -eo pid=,ppid=,pcpu=,args=)
}

class_of() {  # <pid>: CLASS = its own class, else its nearest classified ancestor's, else ''
  local pid=$1 hops=0
  CLASS=''
  while [ -n "$pid" ] && [ "$pid" != 0 ] && [ "$hops" -lt 64 ]; do
    if [ -n "${OWN[$pid]:-}" ]; then
      CLASS=${OWN[$pid]}
      return 0
    fi
    pid=${PARENT[$pid]:-}
    hops=$((hops + 1))
  done
}

pids() {  # the pids of the snapshot, in order
  printf '%s\n' "${!ARGS[@]}" | sort -n
}

dr_processes() {  # <class>: "pid cpu args" of its processes (their own class, or a wrapper's)
  local pid
  snapshot
  for pid in $(pids); do
    class_of "$pid"
    [ "$CLASS" = "$1" ] && printf '%s %s %s\n' "$pid" "${CPU[$pid]}" "${ARGS[$pid]}"
  done
}

plan_stop() {  # STOPPING: the autonomy's pids; GUARDED: autonomy that also runs kept processes
  local pid up hops
  local -A guard=()
  STOPPING=()
  GUARDED=()
  snapshot
  for pid in "${!ARGS[@]}"; do  # the ancestors of what is kept or left alone are not stopped
    class_of "$pid"
    if [ -z "$CLASS" ] || [ "$CLASS" = stop ]; then continue; fi
    up=${PARENT[$pid]:-}
    hops=0
    while [ -n "$up" ] && [ "$up" != 0 ] && [ "$hops" -lt 64 ]; do
      guard[$up]=1
      up=${PARENT[$up]:-}
      hops=$((hops + 1))
    done
  done
  for pid in $(pids); do
    class_of "$pid"
    [ "$CLASS" = stop ] || continue
    if [ -n "${guard[$pid]:-}" ]; then GUARDED+=("$pid"); else STOPPING+=("$pid"); fi
  done
}

list() {  # <class> <title>
  local lines
  lines=$(dr_processes "$1")
  say "$2:"
  if [ -z "$lines" ]; then say "  none"; else printf '%s\n' "$lines" | cut -c1-150 | sed 's/^/  /'; fi
}

stop_all() {  # the autonomy at once: SIGINT (a roslaunch shuts its nodes down), TERM, KILL
  local sig ticks i pid targets='' denied=''
  for sig in INT TERM KILL; do
    plan_stop  # a node whose wrapper is gone stays a target: it is orphaned, not kept
    # shellcheck disable=SC2086
    targets=$(remaining $targets "${STOPPING[@]}")
    [ -n "$targets" ] || return 0
    for pid in $targets; do  # a wrapper and its node together: neither restarts the other
      if ! kill -s "$sig" "$pid" 2>/dev/null && alive "$pid"; then
        case " $denied " in *" $pid "*) ;; *) denied="$denied $pid" ;; esac
      fi
    done
    case "$sig" in INT) ticks=50 ;; TERM) ticks=30 ;; *) ticks=10 ;; esac
    for ((i = 0; i < ticks; i++)); do
      # shellcheck disable=SC2086
      [ -n "$(remaining $targets)" ] || break
      sleep 0.1
    done
  done
  plan_stop
  # shellcheck disable=SC2086
  targets=$(remaining $targets "${STOPPING[@]}")
  [ -n "$targets" ] || return 0
  for pid in $denied; do say "  cannot signal $pid (another user's process): run with sudo"; done
  FAILED=1
}

udp_holder() {  # <port>: free, or the process that has it
  local line
  command -v ss > /dev/null || { printf 'unknown (no ss)'; return 0; }
  line=$(ss -H -ulpn "sport = :$1" 2>/dev/null | head -n 1)
  if [ -z "$line" ]; then
    printf 'free'
  elif [[ $line == *users:* ]]; then
    printf 'in use by %s' "$(sed 's/.*users:((//; s/)).*//' <<< "$line")"
  else
    printf 'in use by another user (sudo ss -ulpn)'
  fi
}

check_host() {
  if [ ! -d "$JY" ]; then
    say "no $JY: run this on the navigation host (192.168.1.106), or set JY_COG"
    exit 1
  fi
  local ips
  ips=$(hostname -I 2>/dev/null || true)
  case " $ips " in
    *" 192.168.1.103 "* | *" 192.168.1.105 "*)
      if [ "$FORCE" = 0 ]; then
        say "this is the motion or the perception host ($ips): nothing to turn off here"
        say "(--force to go on anyway)"
        exit 1
      fi
      ;;
  esac
}

cmd_status() {
  local before='' starters
  [ -f "$BACKUP" ] && before=" (before x30_ros: $(tr -d ' \t\r\n' < "$BACKUP"))"
  say "jy_cog: $JY"
  say "autorun_slam: $(autorun)$before"
  list stop "Deep Robotics autonomy, URDF and bridge (stopped by off)"
  list keep "kept (sensor drivers, ROS master, robot server)"
  list other "other jy_cog processes (left alone)"
  say "load $(cut -d' ' -f1-3 /proc/loadavg) on $(nproc) cores; busiest:"
  ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -n 9 | sed 's/^/  /'
  say "udp $REPORT_PORT (x30_driver's reports): $(udp_holder "$REPORT_PORT")"
  starters=$({
    systemctl list-units --type=service --state=running --no-pager --no-legend 2>/dev/null |
      grep -iE 'jy|ysc|robot|deep|slam|nav' | awk '{print "  systemd: " $1}'
    find "$(home_of_user)/.config/autostart" -maxdepth 1 -name '*.desktop' \
      -printf '  autostart: %f\n' 2>/dev/null
    crontab -l 2>/dev/null | grep '@reboot' | sed 's/^/  cron: /'
    grep -rlsI jy_cog /etc/systemd/system /lib/systemd/system /etc/xdg/autostart /etc/rc.local \
      /etc/profile.d "$(home_of_user)/.config/autostart" "$(home_of_user)/.config/systemd" |
      sed 's/^/  file: /'
  })
  if [ -n "$starters" ]; then say "started at boot by:"; say "$starters"; fi
  return 0
}

cmd_off() {
  local script pid
  say "at boot:"
  case "$(autorun)" in
    true | True | TRUE)
      [ -f "$BACKUP" ] || run cp -p "$CONF" "$BACKUP"
      run sed -i 's/[Tt][Rr][Uu][Ee]/false/' "$CONF"
      say "  autorun_slam false: localization and navigation stay off from the next boot"
      ;;
    missing) say "  no $CONF: nothing set to start" ;;
    *) say "  autorun_slam is $(autorun) already" ;;
  esac
  say "now:"
  for script in "$SCRIPTS/kill_nav.sh" "$SCRIPTS/kill_mapping.sh"; do
    if [ -f "$script" ]; then
      say "  $(basename "$script")"
      run_quiet bash "$script"
    fi
  done
  if [ -f "$RECORDER/stop_record.sh" ] && dr_processes stop | grep -qi record; then
    say "  stop_record.sh"
    run_quiet bash "$RECORDER/stop_record.sh"
  fi
  plan_stop
  for pid in "${STOPPING[@]}"; do say "  stop $pid ${ARGS[$pid]:0:100}"; done
  for pid in "${GUARDED[@]}"; do
    say "  left $pid ${ARGS[$pid]:0:100} (it also runs what is kept)"
  done
  [ "$DRY" = 1 ] && return 0
  stop_all
  list stop "still running"
  list keep "kept"
  return "$FAILED"
}

cmd_sweep() {  # while x30_ros runs (its watchdog, every 30 s): never kill_nav.sh and the like
  local pid
  plan_stop
  for pid in "${STOPPING[@]}"; do say "stop $pid ${ARGS[$pid]:0:100}"; done
  [ "$DRY" = 1 ] && return 0
  stop_all
  return "$FAILED"
}

cmd_on() {
  local was
  if [ -f "$BACKUP" ]; then
    was=$(tr -d ' \t\r\n' < "$BACKUP")
    run mv "$BACKUP" "$CONF"
    say "autorun_slam $was again (as before x30_ros): back from the next boot"
  else
    say "no backup from off: autorun_slam left as $(autorun)"
  fi
  say "to start it now: bash $SCRIPTS/start_nav.sh"
}

main() {
  local command='' arg
  for arg in "$@"; do
    case "$arg" in
      --dry-run) DRY=1 ;;
      --force) FORCE=1 ;;
      status | off | on | sweep) command=$arg ;;
      *) usage ;;
    esac
  done
  [ -n "$command" ] || usage
  check_host
  "cmd_$command"
}

main "$@"
