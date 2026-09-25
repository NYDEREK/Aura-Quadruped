#!/bin/zsh
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
"$root/tools/gazebo-runtime/env/bin/python" "$root/gazebo/generate_aura_quadruped.py"
# Server-only is deliberate: it gives a repeatable physics process on this Mac.
# The current conda OGRE package crashes in the separate graphical client on
# macOS 26, while the DART server and transport interface work normally.
exec "$root/tools/gazebo-runtime/env/bin/gz" sim -s -r -v 3 "$root/gazebo/aura_quadruped.sdf"
