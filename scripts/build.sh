#!/usr/bin/env bash
set -euo pipefail

# repo root (one level up from this script)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null && pwd)"
BUILD_DIR="$REPO_ROOT/build"
NUM_JOBS=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)

mkdir -p "$BUILD_DIR"
pushd "$BUILD_DIR" >/dev/null

# Configure project from repo root
cmake "$REPO_ROOT" -DCMAKE_BUILD_TYPE=Release

# Build the two libraries (graphLib and rrtLib)
cmake --build . --target graphLib -- -j"$NUM_JOBS"
cmake --build . --target rrtLib  -- -j"$NUM_JOBS"

popd >/dev/null

echo "Built graphLib and rrtLib in ${BUILD_DIR}/"