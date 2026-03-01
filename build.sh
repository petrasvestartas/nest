#!/usr/bin/env bash
# Build opennest_solver.dll (C++ CMake, Visual Studio 2022, x64)
# Usage: ./build.sh [--reconfigure]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

echo "=== nest: C++ DLL build ==="
echo "Output: $BUILD_DIR/Release/opennest_solver.dll"
echo ""

# Re-run CMake configure if requested or cache is missing
if [[ "${1:-}" == "--reconfigure" ]] || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "--- cmake configure ---"
    cmake -B "$BUILD_DIR" -G "Visual Studio 17 2022" -A x64 -S "$SCRIPT_DIR"
    echo ""
fi

echo "--- cmake build (Release, target: opennest_solver) ---"
cmake --build "$BUILD_DIR" --config Release --target opennest_solver

echo ""
DLL="$BUILD_DIR/Release/opennest_solver.dll"
if [ -f "$DLL" ]; then
    echo "OK  $DLL"
else
    echo "FAIL: DLL not found at expected path."
    exit 1
fi
