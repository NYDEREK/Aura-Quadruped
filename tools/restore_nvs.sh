#!/bin/bash
# Write a saved NVS image (settings: calibration, gait profiles, CoM, pad
# pairing) back to Aura. The firmware itself is not touched.
#   tools/restore_nvs.sh backups/nvs-before-YYYYMMDD-HHMMSS.bin
cd "$(dirname "$0")/.." || exit 1
IMAGE="$1"
[ -f "$IMAGE" ] && [ "$(wc -c < "$IMAGE")" -eq 24576 ] || { echo "ERROR: need a 24576-byte NVS backup"; exit 1; }
PORT=$(ls /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial* 2>/dev/null | head -1)
[ -n "$PORT" ] || { echo "ERROR: Aura not found on USB"; exit 2; }
ACT="$HOME/.espressif/tools/activate_idf_v6.1.sh"
STAMP=$(date +%Y%m%d-%H%M%S); mkdir -p backups
run() { /bin/bash -c 'source "$1" >/dev/null; shift; eval "$@"' bash "$ACT" "$@"; }
echo "== backup of the current settings =="
run '"$IDF_PYTHON_ENV_PATH/bin/python" -m esptool --chip esp32 -p '"$PORT"' -b 115200 read-flash 0x9000 0x6000 backups/nvs-before-restore-'"$STAMP"'.bin' || exit 3
echo "== writing $IMAGE =="
run '"$IDF_PYTHON_ENV_PATH/bin/python" -m esptool --chip esp32 -p '"$PORT"' -b 115200 write-flash 0x9000 '"$IMAGE"'' || exit 4
echo "== verify =="
run '"$IDF_PYTHON_ENV_PATH/bin/python" -m esptool --chip esp32 -p '"$PORT"' -b 115200 verify-flash 0x9000 '"$IMAGE"'' && echo "== RESTORED =="
