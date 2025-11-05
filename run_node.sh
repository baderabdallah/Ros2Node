#!/usr/bin/env zsh

# Build and launch the ros_node_blueprint package without any external libs.

set -e
set -u
set -o pipefail

SCRIPT_NAME=$(basename "$0")

log() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

usage() {
  cat <<'USAGE'
Build the ROS 2 workspace and launch the BluePrint node.

Usage:
  run_node.sh [options]

Options:
  --build-only         Build and exit, do not launch
  --run-only           Launch without building (expects workspace already built)
  --up-to PKG          Build up to a package (colcon --packages-up-to). Default: ros_node_blueprint
  --select PKG         Build only a package (colcon --packages-select). Overrides --up-to
  -t, --tests-run-dir  Launch with the tests run_dir (src/tests/run under ros_node_blueprint)
  -r, --run-dir DIR    Launch with a specific run_dir path
  -v, --verbose        Verbose build output
  -h, --help           Show this help and exit

Behavior:
- Derives paths relative to this script (no hardcoded absolute paths).
- Uses your current ROS 2 environment if available; otherwise tries /opt/ros/humble, then /opt/ros/jazzy.
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

# Defaults
BUILD_ONLY=0
RUN_ONLY=0
PACKAGES_UP_TO="ros_node_blueprint"
PACKAGES_SELECT=""
USE_TESTS_RUN_DIR=0
CUSTOM_RUN_DIR=""
VERBOSE=0

parse_args() {
  local argv=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --build-only) BUILD_ONLY=1; shift ;;
      --run-only) RUN_ONLY=1; shift ;;
      --up-to) PACKAGES_UP_TO="$2"; shift 2 ;;
      --select) PACKAGES_SELECT="$2"; shift 2 ;;
      -t|--tests-run-dir) USE_TESTS_RUN_DIR=1; shift ;;
      -r|--run-dir) CUSTOM_RUN_DIR="$2"; shift 2 ;;
      -v|--verbose) VERBOSE=1; shift ;;
      -h|--help) usage; exit 0 ;;
      --) shift; break ;;
      -*) err "Unknown option: $1"; usage; exit 2 ;;
      *) argv+=("$1"); shift ;;
    esac
  done
  if [[ ${#argv[@]} -gt 0 ]]; then
    if [[ ${#argv[@]} -eq 1 && "${argv[1]}" == tests ]]; then
      USE_TESTS_RUN_DIR=1
      warn "Positional 'tests' detected: enabling --tests-run-dir (deprecated; use -t/--tests-run-dir)"
    else
      warn "Ignoring unexpected positional args: ${argv[*]}"
    fi
  fi

  if [[ $BUILD_ONLY -eq 1 && $RUN_ONLY -eq 1 ]]; then
    err "--build-only and --run-only cannot be used together"; exit 2
  fi
}

resolve_paths() {
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  PKG_DIR="$SCRIPT_DIR"                                # .../RosNodeBluePrint
  WORKSPACE_DIR="$(cd "$PKG_DIR"/.. && pwd)"         # .../wtg_management_development_vscode_ws
  DEFAULT_TESTS_RUN_DIR="${PKG_DIR}/src/tests/run"
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
    err "ros2 not found in PATH. Please source your ROS 2 environment (e.g., /opt/ros/humble/setup.zsh)."
    exit 3
  fi
}

build_ros_ws() {
  cd "$WORKSPACE_DIR"
  source_ros_underlay

  local colcon_args=(build --symlink-install)
  if [[ -n "$PACKAGES_SELECT" ]]; then
    colcon_args+=(--packages-select "$PACKAGES_SELECT")
  else
    colcon_args+=(--packages-up-to "$PACKAGES_UP_TO")
  fi
  if [[ $VERBOSE -eq 1 ]]; then
    colcon_args+=(--event-handlers console_direct+)
    colcon_args+=(--cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON)
  fi

  log "Running: colcon ${(@q)colcon_args}"
  colcon "${colcon_args[@]}"
}

launch_node() {
  source_ros_underlay

  # Source workspace overlay
  if [[ -f "$WORKSPACE_DIR/install/setup.zsh" ]]; then
    set +u; source "$WORKSPACE_DIR/install/setup.zsh" || true; set -u
  elif [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    set +u; source "$WORKSPACE_DIR/install/setup.bash" || true; set -u
  else
    warn "Workspace overlay not found; attempting to launch anyway."
  fi

  local launch_args=()
  local chosen_run_dir=""
  if [[ -n "$CUSTOM_RUN_DIR" ]]; then
    chosen_run_dir="$CUSTOM_RUN_DIR"
    log "Launching with run_dir (from --run-dir): $chosen_run_dir"
  elif [[ $USE_TESTS_RUN_DIR -eq 1 ]]; then
    chosen_run_dir="$DEFAULT_TESTS_RUN_DIR"
    log "Launching with run_dir (tests dir): $chosen_run_dir"
  else
    chosen_run_dir="$DEFAULT_TESTS_RUN_DIR"
    log "No run_dir provided; defaulting to tests run_dir: $chosen_run_dir"
  fi
  if [[ ! -d "$chosen_run_dir" ]]; then
    err "$chosen_run_dir does not exist (expected run dir). Use -t or -r to point to a valid directory."
    exit 4
  fi
  launch_args+=("run_dir:=$chosen_run_dir")

  ros2 launch ros_node_blueprint blueprint.launch.py "${launch_args[@]}"
}

main() {
  parse_args "$@"
  resolve_paths

  log "Workspace : $WORKSPACE_DIR"
  log "Package   : $PKG_DIR"

  if [[ $RUN_ONLY -eq 0 ]]; then
    build_ros_ws
  else
    log "Skipping build (--run-only)"
  fi

  if [[ $BUILD_ONLY -eq 1 ]]; then
    log "Build completed. Skipping launch (--build-only)."
    exit 0
  fi

  launch_node
}

main "$@"
