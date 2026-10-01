#!/bin/bash
# Build and flash Aura firmware over USB WITHOUT erasing NVS (servo
# calibration, profiles, pad pairing stay). Backs up NVS before and compares
# it after. Used by the Aura app (Firmware tab) and usable from a terminal.
#   tools/flash_firmware.sh [--pull]
set -o pipefail
cd "$(dirname "$0")/.." || exit 1
if [ "$1" = "--pull" ]; then
    echo "== git pull =="
    git pull --ff-only || { echo "ERROR: git pull failed (local changes or no network)"; exit 10; }
fi
echo "== commit: $(git log -1 --format='%h %s' 2>/dev/null) =="
PORT=$(ls /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then echo "ERROR: Aura not found on USB (connect the USB-C cable)"; exit 2; fi
echo "Port: $PORT"
ACT="$HOME/.espressif/tools/activate_idf_v6.1.sh"
[ -f "$ACT" ] || { echo "ERROR: ESP-IDF 6.1 not installed ($ACT)"; exit 3; }
[ -f main/wifi_credentials.h ] || { echo "ERROR: main/wifi_credentials.h missing (copy wifi_credentials.example.h)"; exit 4; }
idf_run() { /bin/bash -c 'source "$1" >/dev/null; shift; eval "$@"' bash "$ACT" "$@"; }
mkdir -p backups
STAMP=$(date +%Y%m%d-%H%M%S)
echo "== 1/4 NVS backup =="
idf_run '"$IDF_PYTHON_ENV_PATH/bin/python" -m esptool --chip esp32 -p '"$PORT"' -b 115200 read-flash 0x9000 0x6000 backups/nvs-before-'"$STAMP"'.bin' || { echo "ERROR: NVS backup failed - nothing was flashed"; exit 5; }
echo "== 2/4 build =="
idf_run '"$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" build' || { echo "ERROR: build failed - nothing was flashed"; exit 6; }
echo "== 3/4 flash (NVS untouched) =="
idf_run '"$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" -p '"$PORT"' flash' || { echo "ERROR: flash failed"; exit 7; }
echo "== 4/4 NVS verify =="
idf_run '"$IDF_PYTHON_ENV_PATH/bin/python" -m esptool --chip esp32 -p '"$PORT"' -b 115200 read-flash 0x9000 0x6000 backups/nvs-after-'"$STAMP"'.bin'
if cmp -s "backups/nvs-before-$STAMP.bin" "backups/nvs-after-$STAMP.bin"; then
    echo "NVS identical - calibration preserved"
else
    echo "NVS differs after boot (the ESP rewrites its radio calibration on start). Backup: backups/nvs-before-$STAMP.bin"
fi
echo "== DONE =="
