#!/usr/bin/env zsh

# Build and run CMake/CTest in a given directory (e.g., a module tests folder).

set -e
set -u
set -o pipefail

log() { echo "[INFO] $*"; }
err() { echo "[ERROR] $*" >&2; }

usage() {
  cat <<'USAGE'
Build and run tests in a standalone CMake tests directory.

Usage:
  run_tests_dir.sh <path-to-tests-dir> [--clean]

Examples:
  run_tests_dir.sh src/core/ModuleOne/tests
  run_tests_dir.sh src/core/ModuleThree/tests --clean
USAGE
}

if [[ $# -lt 1 ]]; then
  usage; exit 2
fi

TEST_DIR="$1"; shift
CLEAN=0
if [[ "${1:-}" == "--clean" ]]; then
  CLEAN=1; shift
fi

if [[ ! -d "$TEST_DIR" ]]; then
  err "Directory not found: $TEST_DIR"; exit 2
fi

BUILD_DIR="$TEST_DIR/_build"
if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then
  log "Cleaning: $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cmake -S "$TEST_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" -- -j$(nproc)
ctest --test-dir "$BUILD_DIR" --output-on-failure
