#!/usr/bin/env bash
# Flash an ArduCopter .apj over USB, with the checks that make it safe.
#
#   ./flash_pixhawk.sh --identify              # read the bootloader, write nothing
#   ./flash_pixhawk.sh --target fmuv3           # fetch if needed, then flash
#   ./flash_pixhawk.sh --file firmware/x.apj --yes
#
# PROPELLERS OFF. Outputs are undefined while the bootloader runs and again
# when fresh firmware boots. Flash on USB power with the flight battery out.
#
# This is nothing like flashing an APM 2.8. There is no avrdude and no ISP:
# the STM32 carries a PX4 bootloader that speaks its own protocol over the
# same USB CDC port the flight code uses, and ArduPilot's uploader.py drives
# it. The uploader also sends the MAVLink reboot-to-bootloader command itself,
# so the board does not need to be unplugged first -- but if it is already
# sitting in the bootloader, or the firmware is not running, replugging USB
# while the uploader waits is the fallback that always works.
#
# Parameters SURVIVE a flash (they live in FRAM, not flash), but a
# major-version change rewrites defaults for parameters it knows about and
# orphans ones that no longer exist. Back up first:
#     ./preload_params.py --backup
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="fmuv3"
CHANNEL="stable"
FILE=""
PORT=""
ASSUME_YES=0
IDENTIFY_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --target) TARGET="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --file) FILE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --identify) IDENTIFY_ONLY=1; shift ;;
    --list-ports) ls -1 /dev/cu.* 2>/dev/null || echo "(no /dev/cu.* at all)"; exit 0 ;;
    --yes|-y) ASSUME_YES=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

PY_BIN="./.venv/bin/python"
[ -x "$PY_BIN" ] || PY_BIN="$(command -v python3)"

# /dev/cu.*, never /dev/tty.*: the tty node blocks on open waiting for a
# carrier-detect that a USB CDC device never asserts, so the uploader would
# appear to hang forever.
if [ -z "$PORT" ]; then
  PORT="$(ls -1 /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
fi
if [ -z "$PORT" ]; then
  echo "No /dev/cu.usbmodem* found." >&2
  echo "  - charge-only USB cable is the usual cause" >&2
  echo "  - plug straight into the Mac, not through a hub" >&2
  echo "  - a board with no firmware shows up for only ~5s per power cycle" >&2
  exit 1
fi
echo "port     $PORT"

HOLDERS="$(lsof -t -w "$PORT" 2>/dev/null | tr '\n' ' ' || true)"
if [ -n "$(echo "$HOLDERS" | tr -d ' ')" ]; then
  echo "ERROR: another process holds $PORT (pids: $HOLDERS)" >&2
  ps -p $(echo "$HOLDERS" | tr ' ' ',' | sed 's/,$//') -o pid=,comm= 2>/dev/null >&2 || true
  echo "Quit QGroundControl / MAVProxy / other tools here first." >&2
  exit 1
fi

if [ "$IDENTIFY_ONLY" = "1" ]; then
  echo
  echo "Reading the bootloader. It will reboot the board; nothing is written."
  exec "$PY_BIN" tools/uploader.py --identify --port "$PORT"
fi

if [ -z "$FILE" ]; then
  FILE="$(ls -1t firmware/arducopter-${TARGET}-*.apj 2>/dev/null | head -1 || true)"
  if [ -z "$FILE" ]; then
    echo "no local image for $TARGET; fetching"
    ./fetch_firmware.sh "$TARGET" "$CHANNEL"
    FILE="$(ls -1t firmware/arducopter-${TARGET}-*.apj | head -1)"
  fi
fi
[ -f "$FILE" ] || { echo "no such file: $FILE" >&2; exit 1; }

echo "image    $FILE"
"$PY_BIN" - "$FILE" <<'PY'
import base64, json, sys
d = json.load(open(sys.argv[1]))
print("board_id %s   summary %s   git %s" % (d["board_id"], d.get("summary"), d["git_identity"]))
print("size     %d bytes (max %s)" % (d["image_size"], d.get("image_maxsize")))
if d["board_id"] != 9:
    print("NOTE: board_id is %s, not 9. Make sure that matches this hardware." % d["board_id"])
PY

if [ "$ASSUME_YES" != "1" ]; then
  echo
  echo "About to overwrite the flight firmware on $PORT."
  printf "Propellers off and battery unplugged? Type yes to flash: "
  read -r reply
  [ "$reply" = "yes" ] || { echo "aborted"; exit 1; }
fi

echo
"$PY_BIN" tools/uploader.py --port "$PORT" "$FILE"
echo
echo "flashed. The board reboots on its own; give it ~15s, then:"
echo "    ./identify.py"
