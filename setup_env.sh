#!/usr/bin/env bash
# Create the Python venv used by every tool here.
#
#   ./setup_env.sh && source .venv/bin/activate
#
# Separate from flashing: flash_cube.sh is pure shell + the ArduPilot
# uploader and needs only a system python3.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null || { echo "python3 not found" >&2; exit 1; }

[ -d .venv ] || "$PY" -m venv .venv
./.venv/bin/pip install --quiet --upgrade pip
./.venv/bin/pip install --quiet -r requirements.txt

echo "ok -- venv ready"
echo "activate with: source $SCRIPT_DIR/.venv/bin/activate"
