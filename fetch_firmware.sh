#!/usr/bin/env bash
# Download and validate an ArduCopter .apj for this board.
#
#   ./fetch_firmware.sh                    # fmuv3, stable
#   ./fetch_firmware.sh Pixhawk1 stable
#   ./fetch_firmware.sh fmuv3 beta
#
# Unlike the APM 2.8 toolkit there is no per-airframe build here: the frame is
# a parameter (FRAME_CLASS/FRAME_TYPE), not part of the firmware, so one image
# flies a quad, a hexa or a Y6. What IS part of the image is the BOARD target,
# and that is what the first argument picks.
#
# Why fmuv3 is the default:
#   board_id 9 covers FMUv2 (1 MB), FMUv3 (2 MB) and the Cube. The difference
#   that matters is flash size and which sensors the hwdef probes. fmuv3
#   probes a superset -- mpu6000, mpu9250, icm20948, icm20602, lsm9ds0, two
#   MS5611s -- so a clone with an unexpected second IMU still comes up. The
#   Pixhawk1 target probes only what a genuine 3DR Pixhawk 1 carries.
#
# A 1 MB board (STM32F427 rev 2 silicon, the old flash bug) CANNOT run these:
# use Pixhawk1-1M or fmuv2. ./flash_pixhawk.sh --identify reads the real flash
# size out of the bootloader, which is the only trustworthy answer.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="${1:-fmuv3}"
CHANNEL="${2:-stable}"
BASE="https://firmware.ardupilot.org/Copter/${CHANNEL}/${TARGET}"

mkdir -p firmware
VERSION="$(curl -fsS --max-time 30 "${BASE}/firmware-version.txt" | tr -d '\r\n' | cut -d- -f1)"
[ -n "$VERSION" ] || { echo "could not read firmware-version.txt from $BASE" >&2; exit 1; }

OUT="firmware/arducopter-${TARGET}-${VERSION}.apj"
echo "target   $TARGET"
echo "channel  $CHANNEL"
echo "version  $VERSION"
echo "url      ${BASE}/arducopter.apj"

if [ -f "$OUT" ]; then
  echo "cached   $OUT"
else
  curl -fsS --max-time 300 -o "$OUT.part" "${BASE}/arducopter.apj"
  mv "$OUT.part" "$OUT"
  echo "saved    $OUT"
fi

# An .apj is JSON: board_id, image_size, image_maxsize and a base64 gzipped
# image. Validating it here means flash_pixhawk.sh never hands a truncated
# download to the bootloader.
python3 - "$OUT" <<'PY'
import base64, hashlib, json, sys
path = sys.argv[1]
raw = open(path, "rb").read()
try:
    d = json.loads(raw)
except ValueError as exc:
    sys.exit("not valid JSON (truncated download?): %s" % exc)
for key in ("board_id", "image", "image_size", "image_maxsize", "git_identity"):
    if key not in d:
        sys.exit("apj is missing %r" % key)
image = base64.b64decode(d["image"])
if len(image) == 0:
    sys.exit("apj carries an empty image")
print("board_id %s   summary %s   git %s" % (d["board_id"], d.get("summary"), d["git_identity"]))
print("image    %d bytes uncompressed, max %s   flash_total %s" % (
    d["image_size"], d.get("image_maxsize"), d.get("flash_total")))
print("sha256   %s" % hashlib.sha256(raw).hexdigest())
if d["image_size"] > int(d.get("image_maxsize") or d["image_size"]):
    sys.exit("image is larger than the target's flash")
PY

{
  echo "$(date -u +%Y-%m-%dT%H:%M:%SZ)  ${TARGET}  ${CHANNEL}  ${VERSION}  $(shasum -a 256 "$OUT" | cut -d' ' -f1)"
} >> firmware/MANIFEST.txt
echo "ok       recorded in firmware/MANIFEST.txt"
