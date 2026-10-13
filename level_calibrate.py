#!/usr/bin/env python3
"""Set the level reference by measurement, and verify it took.

    ./level_calibrate.py
    ./level_calibrate.py --verify-only

This is not the accelerometer calibration. The accel calibration describes
the sensor; this describes how the BOARD sits in the AIRFRAME. It writes
AHRS_TRIM_X/Y, and it is the thing to redo when the board shifts on its
mount, after any crash, or after fitting a new anti-vibration mount.

It is also the one calibration people restore from a backup, which is always
wrong: if the board is now sitting 2 degrees left, the old number describes
an aircraft that no longer exists. Re-measure.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402


def attitude(master, seconds=4.0):
    rolls, pitches = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        rolls.append(math.degrees(msg.roll))
        pitches.append(math.degrees(msg.pitch))
    if not rolls:
        return None
    def mean_sd(values):
        mean = sum(values) / len(values)
        sd = math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
        return mean, sd
    return mean_sd(rolls), mean_sd(pitches), len(rolls)


def report(label, result):
    (roll, roll_sd), (pitch, pitch_sd), n = result
    print("  %-8s roll %+.2f deg (sd %.3f)   pitch %+.2f deg (sd %.3f)   [%d samples]" % (
        label, roll, roll_sd, pitch, pitch_sd, n))
    return roll, pitch, max(roll_sd, pitch_sd)


def main():
    ap = argparse.ArgumentParser(description="level reference calibration")
    ap.add_argument("--port")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--yes", "-y", action="store_true",
                    help="do not prompt. The aircraft must already be sitting "
                         "level and still.")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)
    sys_id, comp = link.target(master)

    print()
    print("The aircraft must be sitting on a LEVEL surface in its normal flying")
    print("attitude, completely still. Whatever this surface's error is becomes")
    print("the aircraft's idea of level, and in flight that error shows up as a")
    print("steady drift in one direction.")
    print()

    before = attitude(master)
    if before is None:
        print("no ATTITUDE messages -- the board is not streaming.", file=sys.stderr)
        return 1
    print("attitude now:")
    roll, pitch, sd = report("before", before)
    for axis in "XY":
        print("  AHRS_TRIM_%s %s" % (axis, link.get_param(master, "AHRS_TRIM_%s" % axis)))

    if sd > 0.3:
        print()
        print("The reading is not steady (sd %.3f deg). Something is rocking it:" % sd)
        print("cable tension, a short landing-gear leg, a soft surface. Settle it")
        print("first -- a level calibration taken while it moves is noise.")
        if not args.verify_only:
            return 1

    if args.verify_only:
        print()
        if abs(roll) < 0.5 and abs(pitch) < 0.5:
            print("verdict: level within 0.5 deg. Nothing to do.")
            return 0
        print("verdict: %+.2f / %+.2f deg off level. Run without --verify-only." % (roll, pitch))
        return 1

    if not args.yes:
        input("Return to calibrate: ")
    else:
        print("--yes: calibrating now. The aircraft must be level and still.")
    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 0, 0, 0, 2, 0, 0)

    # ArduPilot answers a long-running calibration with IN_PROGRESS (5) first
    # and the real result afterwards. Treating the first ack as the verdict
    # reported a refusal on a calibration that was running perfectly well --
    # the same mistake as matching "fail" inside "failsafe".
    ACCEPTED, IN_PROGRESS = 0, 5
    IGNORE_PREFIXES = ("prearm", "arm ", "ekf", "ahrs", "initialising", "gps",
                       "u-blox", "compass variance", "battery")
    ok = False
    deadline = time.time() + 60
    while time.time() < deadline:
        msg = master.recv_match(type=["STATUSTEXT", "COMMAND_ACK"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            if msg.command != link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                continue
            if msg.result == IN_PROGRESS:
                print("  board: calibration in progress")
                continue
            ok = msg.result == ACCEPTED
            if not ok:
                print("  the board refused (result %d)" % msg.result)
            break
        text = link.statustext(msg)
        if not text:
            continue
        if text.lower().startswith(IGNORE_PREFIXES):
            continue
        print("  board: %s" % text)
        low = text.lower()
        if "level" in low and ("success" in low or "complete" in low or "done" in low):
            ok = True
            break
        if "level" in low and "fail" in low:
            print("  the board reported the level calibration failed")
            break

    if not ok:
        print()
        print("level calibration did not complete.")
        return 1

    print()
    print("waiting for the estimator to settle, then re-measuring")
    time.sleep(3)
    after = attitude(master)
    if after is None:
        print("lost telemetry after the calibration.")
        return 1
    roll, pitch, sd = report("after", after)
    for axis in "XY":
        print("  AHRS_TRIM_%s %s" % (axis, link.get_param(master, "AHRS_TRIM_%s" % axis)))

    print()
    if abs(roll) < 0.5 and abs(pitch) < 0.5:
        print("verdict: level within 0.5 deg.")
        return 0
    print("verdict: still %+.2f / %+.2f deg off." % (roll, pitch))
    print("If the surface is genuinely level, the board may be mounted at an")
    print("angle large enough that AHRS_ORIENTATION, not AHRS_TRIM, is the")
    print("right fix -- trim is for small mounting errors, not for a board")
    print("fitted sideways.")
    return 1


if __name__ == "__main__":
    sys.exit(link.run(main))
