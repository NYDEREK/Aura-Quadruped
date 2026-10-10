#!/bin/bash
# Back to a confirmed Aura version: firmware tag + saved settings image.
#   tools/restore_stable.sh                 -> newest restore point (below)
#   tools/restore_stable.sh TAG NVS_IMAGE   -> any other one, see docs/STABILNA-WERSJA.md
cd "$(dirname "$0")/.." || exit 1
TAG=${1:-aura-led-bateria-2026-10-10}
NVS=${2:-backups/nvs-led-bateria-2026-10-10.bin}
[ -f "$NVS" ] || { echo "ERROR: $NVS missing"; exit 1; }
git diff --quiet && git diff --cached --quiet || { echo "ERROR: uncommitted changes - commit or stash first"; exit 1; }
git checkout -q "$TAG" || exit 1
bash tools/flash_firmware.sh || exit 1
bash tools/restore_nvs.sh "$NVS" || exit 1
echo "== Aura is back on $TAG =="
