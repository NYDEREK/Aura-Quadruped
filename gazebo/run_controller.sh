#!/bin/zsh
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
[[ -x "$root/gazebo/build/aura_controller" ]] || "$root/gazebo/build_controller.sh"
exec "$root/gazebo/build/aura_controller" "$@"
