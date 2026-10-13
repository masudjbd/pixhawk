#!/usr/bin/env bash
# Launch MAVProxy as a ground station.
#
#   ./gcs.sh                  # connect to the board over USB
#   ./gcs.sh --udp            # also rebroadcast to 127.0.0.1:14550 for QGC
#
# Unlike the APM 2.8 toolkit, MAVProxy is a convenience here rather than a
# necessity: this board speaks MAVLink 2, so QGroundControl talks to it
# natively and no v1<->v2 bridge is needed. Use whichever you prefer -- but
# only ONE of them at a time, because a serial port has a single owner, and
# QGC's AutoConnect will grab the port behind your back.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

[ -x .venv/bin/mavproxy.py ] || { echo "run ./setup_env.sh first" >&2; exit 1; }

PORT="${PORT:-$(ls -1 /dev/cu.usbmodem* 2>/dev/null | head -1 || true)}"
[ -n "$PORT" ] || { echo "no /dev/cu.usbmodem* found" >&2; exit 1; }

HOLDERS="$(lsof -t -w "$PORT" 2>/dev/null | tr '\n' ' ' || true)"
if [ -n "$(echo "$HOLDERS" | tr -d ' ')" ]; then
  echo "$PORT is held by pid(s): $HOLDERS" >&2
  ps -p "$(echo "$HOLDERS" | tr ' ' ',' | sed 's/,$//')" -o pid=,comm= 2>/dev/null >&2 || true
  exit 1
fi

# An import preflight, so an undeclared MAVProxy dependency surfaces as one
# clear message instead of a traceback at launch.
./.venv/bin/python - <<'PY'
import importlib, sys
missing = [m for m in ("pymavlink", "serial", "future", "gnureadline")
           if not importlib.util.find_spec(m)]
if missing:
    sys.exit("missing modules MAVProxy needs: %s\nrun ./setup_env.sh" % ", ".join(missing))
PY

EXTRA=""
if [ "${1:-}" = "--udp" ]; then
  EXTRA="--out=udp:127.0.0.1:14550"
  echo "rebroadcasting to udp:127.0.0.1:14550 -- point QGC at that, and leave"
  echo "its serial AutoConnect OFF so the two do not fight over the port."
fi

exec ./.venv/bin/mavproxy.py --master="$PORT" --baudrate 115200 $EXTRA
