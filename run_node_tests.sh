#!/usr/bin/env zsh

# Build the workspace and run tests for ros_node_blueprint (no external libs).

set -e
set -u
set -o pipefail

log() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

usage() {
  cat <<'USAGE'
Build the ROS 2 workspace and run tests for the selected packages.

Usage:
  run_node_tests.sh [options]

Options:
  --up-to PKG          Build/tests up to a package (colcon --packages-up-to). Default: ros_node_blueprint
  --select PKG         Build/tests only a package (colcon --packages-select). Overrides --up-to
  -v, --verbose        Verbose build output
  -h, --help           Show this help and exit
USAGE
}

detect_cpus() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  elif getconf _NPROCESSORS_ONLN >/dev/null 2>&1; then
    getconf _NPROCESSORS_ONLN
  else
    echo 4
  fi
}

PACKAGES_UP_TO="ros_node_blueprint"
PACKAGES_SELECT=""
VERBOSE=0

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --up-to) PACKAGES_UP_TO="$2"; shift 2 ;;
      --select) PACKAGES_SELECT="$2"; shift 2 ;;
      -v|--verbose) VERBOSE=1; shift ;;
      -h|--help) usage; exit 0 ;;
      --) shift; break ;;
      -*) err "Unknown option: $1"; usage; exit 2 ;;
      *) warn "Ignoring unexpected positional arg: $1"; shift ;;
    esac
  done
}

resolve_paths() {
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  WORKSPACE_DIR="$(cd "$SCRIPT_DIR"/.. && pwd)"
}

source_ros_underlay() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  set +u
  if [[ -f "/opt/ros/humble/setup.zsh" ]]; then
    source /opt/ros/humble/setup.zsh || true
  elif [[ -f "/opt/ros/jazzy/setup.zsh" ]]; then
    source /opt/ros/jazzy/setup.zsh || true
  fi
  set -u
  if ! command -v ros2 >/dev/null 2>&1; then
    err "ros2 not found in PATH. Please source your ROS 2 environment."; exit 3
  fi
}

main() {
  parse_args "$@"
  resolve_paths
  source_ros_underlay

  cd "$WORKSPACE_DIR"

  local build_args=(build --symlink-install)
  if [[ -n "$PACKAGES_SELECT" ]]; then
    build_args+=(--packages-select "$PACKAGES_SELECT")
  else
    build_args+=(--packages-up-to "$PACKAGES_UP_TO")
  fi
  if [[ $VERBOSE -eq 1 ]]; then
    build_args+=(--event-handlers console_direct+)
    build_args+=(--cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON)
  fi

  log "colcon ${(@q)build_args}"
  colcon "${build_args[@]}"

  local test_args=(test)
  if [[ -n "$PACKAGES_SELECT" ]]; then
    test_args+=(--packages-select "$PACKAGES_SELECT")
  elif [[ -n "$PACKAGES_UP_TO" ]]; then
    test_args+=(--packages-up-to "$PACKAGES_UP_TO")
  fi
  log "colcon ${(@q)test_args}"
  colcon "${test_args[@]}"

  log "colcon test-result --verbose"
  colcon test-result --verbose || true
}

main "$@"
