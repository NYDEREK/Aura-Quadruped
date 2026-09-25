#!/bin/zsh
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
runtime="$root/tools/gazebo-runtime/env"
"$runtime/bin/cmake" -S "$root/gazebo" -B "$root/gazebo/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$runtime"
"$runtime/bin/cmake" --build "$root/gazebo/build" --parallel
