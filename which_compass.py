#!/usr/bin/env python3
"""Which magnetometer is which: settle it by rotating things, not by reading parameters.

    ./which_compass.py

No parameter distinguishes "the IST8310 is on the GPS mast" from "the IST8310
is soldered to the flight controller". COMPASS_EXTERNAL is a claim, not a
measurement -- the firmware guesses it from the bus, and on a clone board the
guess can be wrong in either direction. Getting it backwards means the
automatic orientation fit is applied to the wrong sensor.

The measurement: rotate ONE thing at a time and see which compass instance's
field vector changes. A compass bolted to the mast responds when the mast
turns; one on the board does not.

Instance mapping, which is not obvious:
    compass 1 -> RAW_IMU.xmag/ymag/zmag
    compass 2 -> SCALED_IMU2
    compass 3 -> SCALED_IMU3
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

STREAMS = [(1, "RAW_IMU"), (2, "SCALED_IMU2"), (3, "SCALED_IMU3")]


def capture(master, seconds):
    """Per-instance list of (x, y, z) magnetometer samples."""
    out = {n: [] for n, _ in STREAMS}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type=[name for _, name in STREAMS],
                                blocking=True, timeout=0.5)
        if msg is None:
            continue
        kind = msg.get_type()
        for n, name in STREAMS:
            if kind == name:
                out[n].append((msg.xmag, msg.ymag, msg.zmag))
    return out


def spread(samples):
    """How much the field vector moved, as the largest per-axis span."""
    if len(samples) < 2:
        return 0.0
    spans = []
    for axis in range(3):
        values = [s[axis] for s in samples]
        spans.append(max(values) - min(values))
    return max(spans)


def mean_magnitude(samples):
    if not samples:
        return 0.0
    total = 0.0
    for x, y, z in samples:
        total += math.sqrt(x * x + y * y + z * z)
    return total / len(samples)


def main():
    ap = argparse.ArgumentParser(description="identify which compass is where")
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)

    print()
    present = []
    for n in (1, 2, 3):
        devid = link.get_param(master, "COMPASS_DEV_ID%s" % ("" if n == 1 else n))
        info = link.decode_devid(devid or 0, link.COMPASS_DEVTYPES)
        if info:
            present.append(n)
            ext = link.get_param(
                master, "COMPASS_EXTERNAL" if n == 1 else "COMPASS_EXTERN%d" % n)
            print("compass %d  %s   (firmware claims EXTERNAL=%s)" % (
                n, link.devid_str(info), ext))
    if not present:
        print("no compass detected at all.", file=sys.stderr)
        return 1
    if len(present) == 1:
        print()
        print("Only one compass. This test can still tell you WHERE it is, which")
        print("is the thing no parameter will tell you.")

    baseline = capture(master, 3.0)
    print()
    for n in present:
        print("  compass %d resting field magnitude %.0f mGauss" % (
            n, mean_magnitude(baseline[n])))
        if mean_magnitude(baseline[n]) < 50:
            print("    suspiciously weak -- a healthy magnetometer reads a few")
            print("    hundred mGauss anywhere on earth. Check it is really being read.")

    results = {}
    for label, instruction in (
            ("mast", "Rotate ONLY the GPS mast / compass module.\n"
                     "  Hold the airframe still and turn the module through 180 degrees\n"
                     "  and back. If the module is hard-mounted, skip with Return."),
            ("airframe", "Now rotate the WHOLE AIRFRAME.\n"
                         "  Yaw it through 180 degrees and back, keeping the mast\n"
                         "  in its normal position relative to the frame.")):
        print()
        print("STEP: %s" % instruction)
        input("  Return, then move for the next %.0fs: " % args.seconds)
        samples = capture(master, args.seconds)
        results[label] = {n: spread(samples[n]) for n in present}
        for n in present:
            print("    compass %d field moved by %.0f" % (n, results[label][n]))

    print()
    print("conclusion")
    for n in present:
        mast = results["mast"].get(n, 0.0)
        frame = results["airframe"].get(n, 0.0)
        total = mast + frame
        if total < 100:
            print("  compass %d: barely moved in either test. The test did not" % n)
            print("             exercise it -- move more firmly and retry.")
            continue
        mast_share = 100.0 * mast / total
        if mast_share > 60:
            where = "ON THE MAST (external)"
        elif mast_share < 25:
            where = "ON THE BOARD (internal)"
        else:
            where = "INCONCLUSIVE -- the two moves affected it about equally"
        print("  compass %d: %.0f%% of its movement came from the mast -> %s" % (
            n, mast_share, where))

    print()
    print("What to do with this:")
    print("  - a compass on the mast should have EXTERNAL=1. If the firmware says")
    print("    0, apply profiles/20-compass-mast-primary.params.")
    print("  - a compass on the BOARD that the firmware calls external will have")
    print("    its orientation fitted as though it were on the mast, which is")
    print("    wrong. Set that instance's EXTERN flag to 0 before calibrating.")
    print("  - record the answer in airframe.local.json so ./preflight.py stops")
    print("    asking: no parameter distinguishes a deliberate choice from a")
    print("    mistake, and without the note the warning fires forever on a")
    print("    correctly configured aircraft.")
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
