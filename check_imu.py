#!/usr/bin/env python3
"""Gyro and accelerometer health, by measurement. No battery, no props needed.

    ./check_imu.py              # rest noise, then a guided rotation test
    ./check_imu.py --rest-only  # just the at-rest statistics

Why a rotation test rather than watching it sit still:

RAW_IMU carries the gyro as int16 milli-rad/s, so one count is 0.001 rad/s
(0.057 deg/s). A healthy MPU6000 sitting on a desk is quieter than a single
count, so every sample lands on the same integer and the measured noise is
EXACTLY zero. Zero variance at rest therefore cannot distinguish a quiet gyro
from a dead one -- only a reading pinned far from zero is diagnostic that
way. This cost a day on the APM 2.8 airframe, where a tool "found" two stuck
axes on a perfectly good MPU-6000.

So: measure bias and noise at rest to catch an axis stuck AWAY from zero, and
then rotate the board and check that each axis responds, with the right sign.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

# RAW_IMU units on ArduPilot: accel in mG, gyro in milli-rad/s, mag in mGauss.
G = 9.80665

AXES = [
    ("roll", "x", "roll the aircraft LEFT WING DOWN then back", "xgyro", -1),
    ("pitch", "y", "pitch the NOSE DOWN then back", "ygyro", -1),
    ("yaw", "z", "yaw the NOSE LEFT then back", "zgyro", -1),
]


def sample(master, seconds, kind="RAW_IMU"):
    rows = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type=kind, blocking=True, timeout=0.5)
        if msg is not None:
            rows.append(msg)
    return rows


def stats(values):
    n = len(values)
    if not n:
        return 0.0, 0.0
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return mean, math.sqrt(var)


def main():
    ap = argparse.ArgumentParser(description="IMU health check")
    ap.add_argument("--port")
    ap.add_argument("--rest-only", action="store_true")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)

    print()
    print("Put the aircraft down on a level surface and leave it alone.")
    rows = sample(master, args.seconds)
    if not rows:
        print("no RAW_IMU at all -- the main loop is not streaming.", file=sys.stderr)
        return 1

    print()
    print("at rest, %d samples" % len(rows))
    print("  axis   gyro bias (rad/s)   gyro noise   verdict")
    rest_ok = True
    for name, _, _, field, _ in AXES:
        values = [getattr(r, field) / 1000.0 for r in rows]
        mean, sd = stats(values)
        if abs(mean) > 0.05:
            verdict = "PINNED -- faulty, or the aircraft is moving"
            rest_ok = False
        elif abs(mean) > 0.01:
            verdict = "large bias; re-check it is really still"
        else:
            verdict = "ok"
        print("  %-6s %18.4f %12.4f   %s" % (name, mean, sd, verdict))
    print("  (noise of exactly 0 is normal and proves nothing -- one LSB is")
    print("   0.001 rad/s and this sensor is quieter than that at rest)")

    accels = [(r.xacc / 1000.0 * G, r.yacc / 1000.0 * G, r.zacc / 1000.0 * G) for r in rows]
    mags = [math.sqrt(x * x + y * y + z * z) for x, y, z in accels]
    mean_mag, sd_mag = stats(mags)
    print()
    print("accelerometer magnitude  %.2f m/s/s (sd %.3f)" % (mean_mag, sd_mag))
    if not 8.5 <= mean_mag <= 11.0:
        print("  WRONG. It should read one g (9.81) in any attitude. A magnitude")
        print("  this far off is a dead or saturated axis -- calibration cannot")
        print("  move it by this much, so do not calibrate your way past it.")
        rest_ok = False
    else:
        print("  ok -- one g, whatever attitude the board is in")

    x, y, z = [stats([a[i] for a in accels])[0] for i in range(3)]
    tilt = math.degrees(math.atan2(math.sqrt(x * x + y * y), abs(z))) if z else 90.0
    print("  per-axis  x %+.2f  y %+.2f  z %+.2f   -> tilt from level %.1f deg" % (
        x, y, z, tilt))
    if abs(z) < abs(x) or abs(z) < abs(y):
        print("  NOTE: gravity is not mostly on Z. Either the board is on its side,")
        print("  or it is mounted rotated and AHRS_ORIENTATION needs setting.")

    if args.rest_only:
        return 0 if rest_ok else 1

    print()
    print("Rotation test. Each axis in turn: make a firm, deliberate move and")
    print("bring it back. Press Return, move, then wait for the result.")
    ok = rest_ok
    for name, _, instruction, field, _ in AXES:
        print()
        print("  %s -- %s" % (name.upper(), instruction))
        input("  Return when ready, then move within the 4s window: ")
        rows = sample(master, 4.0)
        peaks = {}
        for other, _, _, other_field, _ in AXES:
            values = [getattr(r, other_field) / 1000.0 for r in rows]
            peaks[other] = max(values, key=abs) if values else 0.0
        mine = peaks[name]
        others = max((abs(v) for k, v in peaks.items() if k != name), default=0.0)
        print("    peak rates: %s" % "  ".join("%s %+.2f" % (k, v) for k, v in peaks.items()))
        if abs(mine) < 0.3:
            print("    NO RESPONSE on %s -- that axis did not move at all." % name)
            print("    Either the move was too gentle, or the axis is dead. Retry")
            print("    harder before concluding anything.")
            ok = False
        elif abs(mine) < others:
            print("    Another axis responded more strongly. The board is probably")
            print("    mounted rotated relative to the airframe -- that is")
            print("    AHRS_ORIENTATION, not a sensor fault.")
            ok = False
        else:
            print("    ok -- %s responded, %.0f%% larger than any other axis" % (
                name, 100.0 * (abs(mine) / max(others, 1e-6) - 1)))

    print()
    print("verdict: %s" % ("IMU responds correctly on all three axes" if ok
                           else "something above needs explaining before flying"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(link.run(main))
