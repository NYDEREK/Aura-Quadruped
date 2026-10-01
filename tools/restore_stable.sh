#!/bin/bash
# Back to the confirmed stable Aura: firmware tag + saved settings image.
cd "$(dirname "$0")/.." || exit 1
TAG=aura-chodzi-2026-10-01
NVS=backups/nvs-dobra-konfiguracja-2026-10-01.bin
[ -f "$NVS" ] || { echo "ERROR: $NVS missing"; exit 1; }
git diff --quiet && git diff --cached --quiet || { echo "ERROR: uncommitted changes - commit or stash first"; exit 1; }
git checkout -q "$TAG" || exit 1
bash tools/flash_firmware.sh || exit 1
bash tools/restore_nvs.sh "$NVS" || exit 1
echo "== Aura is back on $TAG =="
