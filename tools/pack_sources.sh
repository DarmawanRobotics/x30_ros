#!/usr/bin/env bash
# On a computer with internet: x30_ros, the repositories of x30.repos (with their submodules)
# and the Livox SDK in one archive, for a robot without internet (README.md, "Robot without
# internet"). Unpacked in the robot's workspace it gives src/ and Livox-SDK/; src/SOURCES.txt
# lists the commit of every repository. Works on Linux and macOS.
set -u
export COPYFILE_DISABLE=1  # macOS tar: no ._ files

usage() {
  cat <<'EOF'
usage: pack_sources.sh [OUT.tar.gz] [--skip PATH]...
  OUT      the archive (default: x30_sources.tar.gz here)
  --skip   an x30.repos entry the robot has already, e.g. drivers/hikmicro_driver
EOF
  exit 2
}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(dirname "$HERE")
REPOS=${X30_REPOS:-$ROOT/x30.repos}
LIVOX_SDK_URL=${LIVOX_SDK_URL:-https://github.com/Livox-SDK/Livox-SDK.git}
OUT=x30_sources.tar.gz
SKIP=()
STAGE=''

cleanup() { [ -n "$STAGE" ] && rm -rf -- "$STAGE"; }

repos() {  # "path url version" for every entry of a .repos file
  awk '
    /^[[:space:]]*#/ {next}
    /^  [^ ].*:[[:space:]]*$/ {
      if (path != "") print path, url, version
      path = $1; sub(/:$/, "", path); url = ""; version = ""; next
    }
    $1 == "url:" {url = $2}
    $1 == "version:" {version = $2}
    END {if (path != "") print path, url, version}
  ' "$1"
}

skipped() {  # <path>: asked to be left out (the path or its last part)
  local s
  for s in "${SKIP[@]+"${SKIP[@]}"}"; do
    if [ "$s" = "$1" ] || [ "$s" = "${1##*/}" ]; then return 0; fi
  done
  return 1
}

clone() {  # <url> <version> <dir>: shallow, submodules too (whole ones if a server refuses)
  local args=(--quiet --depth 1 --recurse-submodules)
  [ -n "$2" ] && args+=(--branch "$2")
  git clone "${args[@]}" --shallow-submodules "$1" "$3" 2> /dev/null && return 0
  rm -rf -- "$3"
  git clone "${args[@]}" "$1" "$3"
}

main() {
  local path url version
  while [ $# -gt 0 ]; do
    case "$1" in
      --skip) SKIP+=("${2:?--skip needs a path}"); shift ;;
      -*) usage ;;
      *) OUT=$1 ;;
    esac
    shift
  done
  command -v git > /dev/null || { echo "git is needed here (not on the robot)"; exit 1; }
  STAGE=$(mktemp -d)
  trap cleanup EXIT
  mkdir -p "$STAGE/src/x30_ros"
  # the working tree as it is, uncommitted changes included
  tar -C "$ROOT" --exclude=.git --exclude=__pycache__ -cf - . | tar -C "$STAGE/src/x30_ros" -xf -
  {
    echo "# packed by x30_ros/tools/pack_sources.sh, $(date -u '+%Y-%m-%d %H:%M UTC')"
    echo "x30_ros $(git -C "$ROOT" describe --always --dirty 2> /dev/null || echo 'not a git checkout')"
  } > "$STAGE/src/SOURCES.txt"
  echo "x30_ros: $ROOT"
  while read -r path url version; do
    if skipped "$path"; then
      echo "$path: skipped"
      continue
    fi
    echo "$path: $url ${version:-(default branch)}"
    clone "$url" "$version" "$STAGE/src/$path" || { echo "cannot clone $url"; exit 1; }
    echo "$path $url ${version:--} $(git -C "$STAGE/src/$path" rev-parse HEAD)" >> "$STAGE/src/SOURCES.txt"
  done < <(repos "$REPOS")
  echo "Livox-SDK: $LIVOX_SDK_URL"
  clone "$LIVOX_SDK_URL" '' "$STAGE/Livox-SDK" || { echo "cannot clone $LIVOX_SDK_URL"; exit 1; }
  echo "Livox-SDK $LIVOX_SDK_URL - $(git -C "$STAGE/Livox-SDK" rev-parse HEAD)" >> "$STAGE/src/SOURCES.txt"
  # no git history (the robot cannot fetch anyway) and no FAST_LIO videos: ~10 MB, not 280
  find "$STAGE" -name .git -prune -exec rm -rf {} +
  rm -rf -- "$STAGE/src/third_party/FAST_LIO/doc"
  tar -C "$STAGE" -czf "$OUT" src Livox-SDK || exit 1
  cat <<EOF
packed $OUT ($(du -h "$OUT" | cut -f1)). To the robot, e.g. its navigation host:
  scp $OUT ysc@192.168.1.106:~/
and there, into the workspace (~/hotdog here):
  mkdir -p ~/hotdog && tar -xzf ~/$(basename "$OUT") -C ~/hotdog
  bash ~/hotdog/src/x30_ros/tools/check_deps.sh
EOF
}

main "$@"
