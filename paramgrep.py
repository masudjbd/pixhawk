#!/usr/bin/env python3
"""Print the board's parameters whose name matches a pattern.

    ./paramgrep.py GPS              # everything with GPS in the name
    ./paramgrep.py '^COMPASS_.*ID'  # regex
    ./paramgrep.py --nonzero BATT

Exists because parameter names move between ArduPilot releases -- GPS_TYPE
became GPS1_TYPE in 4.6, BATT_FS_LOW_ACT became BATT_FS_LOW_ACT in one
release and BATT_LOW_... in another -- and a tool that hard-codes the old
name silently reports a parameter as absent. Ask the board.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pattern", help="substring or regex, case-insensitive")
    ap.add_argument("--port")
    ap.add_argument("--nonzero", action="store_true", help="hide parameters that are 0")
    args = ap.parse_args()

    try:
        master = link.connect(args.port, quiet=True)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    params = link.fetch_params(master, quiet=True)
    params.pop("_count", None)

    rx = re.compile(args.pattern, re.I)
    hits = sorted(n for n in params if rx.search(n))
    for name in hits:
        value = params[name]
        if args.nonzero and value == 0:
            continue
        print("%-24s %g" % (name, value))
    print("# %d of %d parameters matched %r" % (len(hits), len(params), args.pattern))
    return 0


if __name__ == "__main__":
    sys.exit(main())
