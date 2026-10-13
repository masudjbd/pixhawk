#!/usr/bin/env bash
# Static check everything here. Not needed to fly; needed to trust an edit.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

PY="./.venv/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"

fail=0
echo "== pyflakes =="
"$PY" -m pyflakes ./*.py lib/*.py tests/*.py 2>/dev/null || fail=1

echo "== compile =="
"$PY" -m py_compile ./*.py lib/*.py || fail=1

echo "== shellcheck (if installed) =="
if command -v shellcheck >/dev/null; then
  shellcheck ./*.sh || fail=1
else
  echo "(shellcheck not installed; skipping)"
fi

echo "== profiles parse =="
"$PY" - <<'PY' || fail=1
import glob, re, sys
bad = 0
for path in sorted(glob.glob("profiles/*.params")):
    for lineno, line in enumerate(open(path), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = re.split(r"[,\s]+", line, maxsplit=1)
        if len(parts) != 2:
            print("%s:%d: not NAME,VALUE: %r" % (path, lineno, line)); bad = 1
            continue
        try:
            float(parts[1])
        except ValueError:
            print("%s:%d: value is not a number: %r" % (path, lineno, line)); bad = 1
sys.exit(bad)
PY

[ "$fail" = 0 ] && echo "ok" || echo "problems above"
exit "$fail"
