#!/usr/bin/env bash
# NeuPAN for x30_navigation's neupan controller, in a Python venv beside ROS: PyTorch (CPU),
# cvxpy, cvxpylayers and NeuPAN (its py38 branch) with the X30 patches of tools/neupan (the
# holonomic kinematics: forward and sideways speed and the yaw rate), at versions that run on
# Python 3.8 (Noetic). The venv sees the system's and ROS's packages; the path follower runs
# with its python through X30_PYTHON (x30_navigation README, "NeuPAN"). For the navigation host
# (192.168.1.106): with internet, or with the wheels --download fetched on another computer.
set -euo pipefail

usage() {
  cat <<'EOF'
usage: install_neupan.sh [--venv DIR] [--wheels DIR]   install (or update) the venv
       install_neupan.sh --download DIR                every wheel into DIR (internet, any OS)
       install_neupan.sh --check [--venv DIR]          what the venv has
  --venv DIR    the venv (default: ~/x30_ws/neupan)
  --wheels DIR  install from DIR only, no internet (made by --download)
EOF
  exit 2
}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
VENV=${HOME}/x30_ws/neupan
NEUPAN_URL=${NEUPAN_URL:-https://github.com/hanruihua/NeuPAN.git}
NEUPAN_COMMIT=70991f0d96b7a15135ba02ea8ad3f3090ff0926e # py38: "Add python 3.8 compatibility"
TORCH=torch==2.4.1                                      # the last for Python 3.8
TORCH_INDEX=https://download.pytorch.org/whl/cpu        # CPU only: no CUDA, 200 MB, not 2.5 GB
PINNED=(
  'numpy==1.24.4' 'scipy==1.10.1' 'cvxpy==1.5.2' 'cvxpylayers==0.1.6' 'diffcp==1.0.23'
  'ecos==2.0.14' 'gctl==1.2' 'matplotlib==3.7.5' rich colorama dill pyyaml
)
# a newer setuptools or numpy in ~/.local must not leak into the venv (check_deps.sh)
export PYTHONNOUSERSITE=1 PIP_DISABLE_PIP_VERSION_CHECK=1

say() { printf '%s\n' "$*"; }

neupan_wheel() { # <dir> <python>: NeuPAN at the pinned commit, X30 patches applied, a wheel
  local out=$1 python=$2 src
  src=$(mktemp -d)
  git clone -q "$NEUPAN_URL" "$src"
  git -C "$src" -c advice.detachedHead=false checkout -q "$NEUPAN_COMMIT"
  git -C "$src" apply "$HERE"/neupan/*.patch
  "$python" -m pip wheel -q --no-deps -w "$out" "$src"
  rm -rf "$src"
}

check() {
  local py=$VENV/bin/python
  if [ ! -x "$py" ]; then
    say "no venv at $VENV: tools/install_neupan.sh"
    return 1
  fi
  "$py" - <<'EOF'
import sys
import cvxpy
import cvxpylayers
import numpy
import torch
from neupan.robot import robot

holonomic = robot(kinematics='holonomic', length=1.0, width=0.5, max_speed=[1, 1, 1],
                  max_acce=[1, 1, 1])
assert holonomic.control_dim == 3, 'NeuPAN without the X30 patches'
print(f'python {sys.version.split()[0]}, torch {torch.__version__}, cvxpy {cvxpy.__version__}, '
      f'cvxpylayers {cvxpylayers.__version__}, numpy {numpy.__version__}')
print('NeuPAN with the holonomic kinematics: ok')
EOF
}

download() { # <dir>
  local dir=$1 tags=(--only-binary=:all: --python-version 3.8 --implementation cp --abi cp38)
  mkdir -p "$dir"
  python3 -m pip download -q -d "$dir" "${tags[@]}" --platform manylinux2014_x86_64 \
    'pip<25' setuptools wheel "${PINNED[@]}"
  python3 -m pip download -q -d "$dir" "${tags[@]}" --platform linux_x86_64 \
    --platform manylinux2014_x86_64 --index-url "$TORCH_INDEX" \
    --extra-index-url https://pypi.org/simple "$TORCH"
  neupan_wheel "$dir" python3
  say "wheels for Python 3.8 on x86_64 in $dir: copy it to the robot, then"
  say "  bash tools/install_neupan.sh --wheels $dir"
}

install() { # [wheels dir]
  local wheels=${1:-} pip
  if [ ! -x "$VENV/bin/python" ]; then
    if ! python3 -m venv --system-site-packages "$VENV"; then
      say "no venv module: sudo apt install python3-venv (then again)"
      return 1
    fi
  fi
  pip=("$VENV/bin/python" -m pip)
  if [ -n "$wheels" ]; then
    pip+=(install -q --no-index --find-links "$wheels")
    "${pip[@]}" -U 'pip<25'
    "${pip[@]}" "$TORCH" "${PINNED[@]}" neupan
  else
    "${pip[@]}" install -q -U 'pip<25' setuptools wheel
    if [ "$(uname -m)" = x86_64 ]; then
      "${pip[@]}" install -q "$TORCH" --index-url "$TORCH_INDEX"
    else
      "${pip[@]}" install -q "$TORCH" # PyPI's wheels for ARM are CPU only already
    fi
    "${pip[@]}" install -q "${PINNED[@]}"
    local built
    built=$(mktemp -d)
    neupan_wheel "$built" "$VENV/bin/python"
    "${pip[@]}" install -q --no-deps --force-reinstall "$built"/neupan-*.whl
    rm -f "$built"/neupan-*.whl
    rmdir "$built"
  fi
  check
  say "the path follower uses it through X30_PYTHON (in ~/.bashrc and the stack's service):"
  say "  export X30_PYTHON=$VENV/bin/python"
}

main() {
  local mode=install wheels='' dir=''
  while [ $# -gt 0 ]; do
    case "$1" in
      --venv) VENV=${2:?--venv needs a folder}; shift ;;
      --wheels) wheels=${2:?--wheels needs a folder}; shift ;;
      --download) mode=download; dir=${2:?--download needs a folder}; shift ;;
      --check) mode=check ;;
      *) usage ;;
    esac
    shift
  done
  case "$mode" in
    download) download "$dir" ;;
    check) check ;;
    install) install "$wheels" ;;
  esac
}

main "$@"
