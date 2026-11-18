#!/usr/bin/env bash
# Deep Robotics' own autonomy on the X30's navigation host (192.168.1.106, ~/jy_cog): its
# localization, navigation and mapping take the CPU x30_ros needs and clash with it (/map,
# /initialpose, TF, /cmd_vel). Turns it off and on again; offline, on the navigation host.
# Kept: the sensor drivers (/livox/lidar, /imu/data), the ROS master and the robot server
# (the controller app). Never touched: the motion host (.103) and the perception host (.105:
# elevation map, autonomous charging and their drivers).
set -u

usage() {
  cat <<'EOF'
usage: dr_services.sh status|off|on [--dry-run] [--force]
  status   autorun_slam, the Deep Robotics processes, the busiest processes
  off      stop the localization, navigation and mapping now, and keep them off at boot
  on       autorun_slam as it was before off (back from the next boot)
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
# rest is only listed
STOP='locali[sz]ation|location|navigation|(^|/)nav|move_base|slam|mapping|player|record|rviz|pcd2map|occ_'
KEEP='livox|lidar|yesense|(^|[/_])imu|driver|robot_server|broker|rosmaster|roscore|rosout'
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
  local state
  state=$(ps -o stat= -p "$1" 2>/dev/null) || return 1
  case "$state" in Z* | '') return 1 ;; esac
  return 0
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

dr_processes() {  # <class>: "pid cpu args" of the jy_cog processes of that class
  local pid cpu args class
  while read -r pid cpu args; do
    [ "$pid" = "$$" ] && continue
    program "$args"
    [ -n "$PROGRAM" ] || continue
    if [[ $PROGRAM =~ $KEEP ]]; then
      class=keep
    elif [[ $PROGRAM =~ $STOP ]]; then
      class=stop
    else
      class=other
    fi
    [ "$class" = "$1" ] && printf '%s %s %s\n' "$pid" "$cpu" "$args"
  done < <(ps -eo pid=,pcpu=,args=)
}

list() {  # <class> <title>
  local lines
  lines=$(dr_processes "$1")
  say "$2:"
  if [ -z "$lines" ]; then say "  none"; else printf '%s\n' "$lines" | cut -c1-150 | sed 's/^/  /'; fi
}

stop_pid() {  # <pid>: SIGINT (a roslaunch shuts its nodes down), then TERM, then KILL
  local pid=$1 sig ticks i
  for sig in INT TERM KILL; do
    if ! kill -s "$sig" "$pid" 2>/dev/null; then
      alive "$pid" || return 0
      say "  cannot signal $pid (another user's process): run with sudo"
      FAILED=1
      return 0
    fi
    case "$sig" in INT) ticks=50 ;; TERM) ticks=30 ;; *) ticks=10 ;; esac
    for ((i = 0; i < ticks; i++)); do
      alive "$pid" || return 0
      sleep 0.1
    done
  done
  say "  $pid is still running"
  FAILED=1
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
  list stop "Deep Robotics autonomy (stopped by off)"
  list keep "kept (sensor drivers, ROS master, robot server)"
  list other "other jy_cog processes (left alone)"
  say "load $(cut -d' ' -f1-3 /proc/loadavg) on $(nproc) cores; busiest:"
  ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -n 9 | sed 's/^/  /'
  starters=$({
    systemctl list-units --type=service --state=running --no-pager --no-legend 2>/dev/null |
      grep -iE 'jy|ysc|robot|deep|slam|nav' | awk '{print "  systemd: " $1}'
    find "$(home_of_user)/.config/autostart" -maxdepth 1 -name '*.desktop' \
      -printf '  autostart: %f\n' 2>/dev/null
    crontab -l 2>/dev/null | grep '@reboot' | sed 's/^/  cron: /'
  })
  if [ -n "$starters" ]; then say "started at boot by:"; say "$starters"; fi
  return 0
}

cmd_off() {
  local script pid args
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
  while read -r pid _ args; do
    [ -n "$pid" ] || continue
    say "  stop $pid ${args:0:100}"
    [ "$DRY" = 1 ] || stop_pid "$pid"
  done < <(dr_processes stop)
  [ "$DRY" = 1 ] && return 0
  list stop "still running"
  list keep "kept"
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
      status | off | on) command=$arg ;;
      *) usage ;;
    esac
  done
  [ -n "$command" ] || usage
  check_host
  "cmd_$command"
}

main "$@"
