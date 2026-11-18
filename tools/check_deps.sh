#!/usr/bin/env bash
# What x30_ros needs on a robot without internet, checked offline: the Ubuntu 20.04 / ROS Noetic
# packages (dpkg), the source packages of x30.repos in the workspace and the Livox SDK. Prints
# what is missing and one apt line for it; installs nothing (README.md, "Robot without internet").
set -u

usage() {
  cat <<'EOF'
usage: check_deps.sh [--tests] [--gui] [--src DIR]
  --tests   also what catkin_make run_tests needs
  --gui     also RViz and the joint state publisher GUI (display.launch)
  --src     the workspace's src (default: the folder holding x30_ros)
EOF
  exit 2
}

# x30_ros, FAST_LIO, livox_ros_driver, hikmicro_driver and hertzino_driver on Ubuntu 20.04:
# rosdep keys --from-paths src --ignore-src | rosdep resolve --os=ubuntu:focal (tests apart)
BUILD=(
  build-essential cmake git pkg-config
  ros-noetic-catkin ros-noetic-roscpp ros-noetic-rospy ros-noetic-rosbash ros-noetic-roslaunch
  ros-noetic-rosgraph ros-noetic-genpy ros-noetic-message-generation ros-noetic-message-runtime
  ros-noetic-std-msgs ros-noetic-std-srvs ros-noetic-geometry-msgs ros-noetic-sensor-msgs
  ros-noetic-nav-msgs ros-noetic-diagnostic-msgs ros-noetic-visualization-msgs
  ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-tf ros-noetic-tf2-ros
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-xacro
  ros-noetic-topic-tools ros-noetic-rosbag ros-noetic-pcl-ros
  ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-image-transport-plugins
  python3-numpy python3-scipy python3-yaml python3-rospkg python3-setuptools python3-opencv
  libeigen3-dev libpcl-dev libapr1-dev libaprutil1-dev
  libcurl4-openssl-dev libssl-dev libopencv-dev libtinyxml2-dev nlohmann-json3-dev
  gstreamer1.0-libav gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
)
TESTS=(python3-dev python3-pytest python3-nose ros-noetic-rostest ros-noetic-rosunit)
GUI=(ros-noetic-rviz ros-noetic-joint-state-publisher-gui)
# source packages: name, then the x30.repos entry that brings it
SOURCES=(
  "fast_lio third_party/FAST_LIO"
  "livox_ros_driver third_party/livox_ros_driver"
  "hikmicro_msgs drivers/hikmicro_driver"
  "hikmicro_driver drivers/hikmicro_driver"
  "hertzino_msgs drivers/hertzino_driver"
  "hertzino_driver drivers/hertzino_driver"
)
LIVOX_SDK_LIB=${LIVOX_SDK_LIB:-/usr/local/lib/liblivox_sdk_static.a}
ROS_DIR=${ROS_DIR:-/opt/ros/noetic}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC=$(dirname "$(dirname "$HERE")")
PACKAGES=("${BUILD[@]}")
MISSING=()
PROBLEMS=0
BUILD_ENV=''

row() { printf '%-8s %-5s %s\n' "$1" "$2" "$3"; }

check_system() {
  local codename='?' arch
  arch=$(uname -m)
  if [ -r /etc/os-release ]; then
    codename=$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-?}")
  fi
  if [ "$codename" = focal ]; then
    row system ok "Ubuntu 20.04 (focal), $arch"
  else
    row system WARN "Ubuntu $codename, $arch: the list is for 20.04 (focal), as ROS Noetic"
  fi
  if [ -f "$ROS_DIR/setup.bash" ]; then
    row system ok "ROS at $ROS_DIR"
  else
    row system MISS "no $ROS_DIR/setup.bash: ROS Noetic first (ros-noetic-ros-base)"
    PROBLEMS=1
  fi
}

check_apt() {
  if ! command -v dpkg-query > /dev/null; then
    row apt MISS "no dpkg-query: not a Debian or Ubuntu system"
    PROBLEMS=1
    return
  fi
  local installed package
  installed=$(dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n' "${PACKAGES[@]}" 2>/dev/null |
    awk '$2 ~ /^ii/ {print $1}')
  for package in "${PACKAGES[@]}"; do
    if ! grep -qxF "$package" <<< "$installed"; then
      MISSING+=("$package")
    fi
  done
  row apt "$([ ${#MISSING[@]} = 0 ] && printf ok || printf MISS)" \
    "$((${#PACKAGES[@]} - ${#MISSING[@]})) of ${#PACKAGES[@]} installed"
  for package in "${MISSING[@]}"; do row apt MISS "$package"; done
  [ ${#MISSING[@]} = 0 ] || PROBLEMS=1
}

package_dirs() {  # <name>: the folders under SRC holding a package.xml of that name
  grep -rlsE "<name>[[:space:]]*$1[[:space:]]*</name>" --include=package.xml "$SRC" |
    xargs -r -n1 dirname | sed "s|^$SRC/|src/|"
}

check_sources() {
  local entry name repo dirs count
  for entry in "${SOURCES[@]}"; do
    read -r name repo <<< "$entry"
    dirs=$(package_dirs "$name")
    count=$(grep -c . <<< "$dirs")
    if [ "$count" = 0 ]; then
      row sources MISS "$name: put $repo of x30.repos into src (tools/pack_sources.sh)"
      PROBLEMS=1
    elif [ "$count" -gt 1 ]; then
      row sources DUP "$name in $(tr '\n' ' ' <<< "$dirs")(catkin needs one: remove one copy)"
      PROBLEMS=1
    else
      row sources ok "$name ($dirs)"
    fi
  done
  dirs=$(package_dirs fast_lio)
  if [ "$(grep -c . <<< "$dirs")" = 1 ] && [ ! -f "${dirs/#src/$SRC}/include/ikd-Tree/ikd_Tree.cpp" ]
  then
    row sources MISS "FAST_LIO without its ikd-Tree submodule: clone it with --recursive"
    PROBLEMS=1
  fi
}

check_livox_sdk() {
  if [ -f "$LIVOX_SDK_LIB" ]; then
    row livox ok "Livox SDK ($LIVOX_SDK_LIB)"
  else
    row livox MISS "Livox SDK ($LIVOX_SDK_LIB): build it, README.md \"Robot without internet\""
    PROBLEMS=1
  fi
}

check_python() {  # pip --user packages shadow the system's: a newer setuptools there stops catkin
  local site  # catkin's PYTHONPATH holds no setuptools: the user site comes before the system's
  PYTHONPATH='' python3 -c 'import setuptools' > /dev/null 2>&1 && return 0
  if PYTHONPATH='' python3 -s -c 'import setuptools' > /dev/null 2>&1; then
    site=$(python3 -s -c 'import site; print(site.getusersitepackages())' 2>/dev/null)
    row python WARN "setuptools in ${site:-the user site} breaks catkin: PYTHONNOUSERSITE=1"
    BUILD_ENV='PYTHONNOUSERSITE=1 '
  else
    row python MISS "python3 cannot import setuptools (python3-setuptools)"
    PROBLEMS=1
  fi
}

check_rosdep() {  # authoritative when the robot has a rosdep cache already (it reads no network)
  command -v rosdep > /dev/null || return 0
  [ -d "${ROS_HOME:-$HOME/.ros}/rosdep/sources.cache" ] || return 0
  local out line
  if out=$(rosdep check --from-paths "$SRC" --ignore-src -r 2>&1); then
    row rosdep ok "rosdep check: every dependency of src installed"
  else
    row rosdep MISS "rosdep check:"
    while IFS= read -r line; do printf '                %s\n' "$line"; done <<< "$out" | head -n 20
    PROBLEMS=1
  fi
}

main() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --tests) PACKAGES+=("${TESTS[@]}") ;;
      --gui) PACKAGES+=("${GUI[@]}") ;;
      --src) SRC=${2:?--src needs a folder}; shift ;;
      *) usage ;;
    esac
    shift
  done
  SRC=$(cd "$SRC" && pwd) || exit 2
  printf 'x30_ros dependencies, checked offline (src: %s)\n' "$SRC"
  check_system
  check_apt
  check_sources
  check_livox_sdk
  check_python
  check_rosdep
  if [ "$PROBLEMS" = 0 ]; then
    printf 'everything there: %scatkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release\n' \
      "$BUILD_ENV"
    return 0
  fi
  printf 'missing: README.md, "Robot without internet", tells how to bring it in\n'
  if [ ${#MISSING[@]} -gt 0 ]; then
    printf '  sudo apt install -y %s\n' "${MISSING[*]}"
  fi
  return 1
}

main "$@"
