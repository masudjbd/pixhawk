#!/usr/bin/env python3
"""Measure RCn_MIN/MAX/TRIM from live stick travel, and write them verified.

    ./calibrate_radio.py              # measure and report
    ./calibrate_radio.py --write      # measure, then write

Two refusals built in, both learned the hard way on the APM 2.8 airframe:

  NEVER with the transmitter off. On signal loss ch3 falls to ~900 us.
  Calibrating then writes RC3_MIN 900, which is BELOW FS_THR_VALUE -- and
  that silently destroys the throttle failsafe you are relying on. This tool
  refuses to run if no channel is decoding, and refuses to write a RC3_MIN
  that sits below the configured failsafe threshold.

  A partial sweep is not a calibration. A connectivity floor of 200 us is
  enough to prove a channel is alive, and is nowhere near enough to call an
  endpoint measured: using it as the stop condition once wrote RC1_MIN 1333 /
  RC1_MAX 1648 -- a third of real travel -- which would have permanently
  crippled roll authority. The requirement here is 800 us on the four control
  channels.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

CONTROL_CHANNELS = (1, 2, 3, 4)
REQUIRED_TRAVEL = 800
CONNECT_FLOOR = 200


def main():
    ap = argparse.ArgumentParser(description="radio endpoint calibration")
    ap.add_argument("--port")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--channels", type=int, default=8)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="auto-stop after this long (default: wait for Ctrl-C)")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)

    fs_value = link.get_param(master, "FS_THR_VALUE")

    # Capture the trims FIRST, from the sticks at rest, the way Mission
    # Planner does it. Taking them at the END instead means whatever position
    # the sticks happen to be in when sampling stops becomes "centre" -- which
    # is fine when a human presses Ctrl-C with the sticks centred, and wrong
    # whenever the run is time-boxed.
    print()
    print("Step 1: LEAVE THE STICKS ALONE for 4 seconds -- centred, throttle DOWN.")
    print("        This is where the trims come from.")
    trim_rest = {}
    rest_samples = 0
    rest_end = time.time() + 4.0
    acc = {}
    while time.time() < rest_end:
        msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
        if msg is None:
            continue
        rest_samples += 1
        for n in range(1, args.channels + 1):
            v = getattr(msg, "chan%d_raw" % n, 0)
            if v in (0, 65535):
                continue
            acc.setdefault(n, []).append(v)
    for n, values in acc.items():
        trim_rest[n] = int(round(sum(values) / len(values)))
    if trim_rest:
        print("        resting: %s" % "  ".join("ch%d=%d" % (n, trim_rest[n])
                                                for n in sorted(trim_rest)))
    else:
        print("        no RC data -- check the link before going on.")

    print()
    print("Step 2: Transmitter ON. Move every stick and switch to BOTH extremes:")
    print("  - roll and pitch: full left/right, full forward/back")
    print("  - throttle: all the way down, all the way up")
    print("  - yaw: full left, full right")
    print("  - every switch you use, through all its positions")
    print()
    print("Then centre the sticks, leave the throttle DOWN, and press Ctrl-C.")
    print()

    lo, hi, last = {}, {}, {}
    samples = 0
    start = time.time()
    try:
        while True:
            if args.seconds and time.time() - start > args.seconds:
                break
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
            if msg is None:
                continue
            samples += 1
            row = []
            for n in range(1, args.channels + 1):
                value = getattr(msg, "chan%d_raw" % n, 0)
                if value in (0, 65535):
                    row.append("  ch%d ----" % n)
                    continue
                lo[n] = min(lo.get(n, value), value)
                hi[n] = max(hi.get(n, value), value)
                last[n] = value
                row.append("  ch%d %4d [%4d-%4d]" % (n, value, lo[n], hi[n]))
            if samples % 5 == 0:
                print("\r" + "".join(row[:6]), end="", flush=True)
    except KeyboardInterrupt:
        print()

    print()
    if not lo:
        print("No channel ever decoded. REFUSING to calibrate.")
        print()
        print("Calibrating with no signal is how a throttle failsafe gets")
        print("destroyed: on loss the receiver puts out ~900 us, and that would")
        print("be written as RC3_MIN. Connect the receiver, switch the")
        print("transmitter on, and try again.")
        return 1

    print("measured over %d samples:" % samples)
    print("  ch   min   max  travel  now   verdict")
    ok = True
    for n in sorted(lo):
        travel = hi[n] - lo[n]
        if n in CONTROL_CHANNELS:
            if travel >= REQUIRED_TRAVEL:
                verdict = "full sweep"
            elif travel >= CONNECT_FLOOR:
                verdict = "PARTIAL -- not enough to set an endpoint"
                ok = False
            else:
                verdict = "barely moved"
                ok = False
        else:
            verdict = "switch/aux" if travel >= CONNECT_FLOOR else "not moved"
        print("  %-4d %5d %5d %6d %5d  %s" % (
            n, lo[n], hi[n], travel, last.get(n, 0), verdict))

    missing = [n for n in CONTROL_CHANNELS if n not in lo]
    if missing:
        print()
        print("control channels that never decoded: %s" % ", ".join(
            "ch%d" % n for n in missing))
        print("Check RCMAP_ROLL/PITCH/THROTTLE/YAW and the transmitter's channel")
        print("assignment before going further.")
        ok = False

    if fs_value and 3 in lo and lo[3] <= fs_value:
        print()
        print("REFUSING: the lowest throttle seen (%d) is at or below" % lo[3])
        print("FS_THR_VALUE (%g). Writing this would make the throttle failsafe" % fs_value)
        print("fire in normal flight, or never fire at all.")
        print("Either the transmitter was off for part of this run, or the")
        print("throttle endpoint needs trimming on the radio.")
        ok = False

    if not ok:
        print()
        print("not writing. Fix the above and run again.")
        return 1

    if not args.write:
        print()
        print("looks good. Re-run with --write to store it.")
        return 0

    print()
    failed = []
    skipped = []
    for n in sorted(lo):
        # A channel that never moved was not measured. Writing MIN = MAX from
        # it stores a zero-range endpoint built from one sample -- the same
        # class of mistake as calibrating on a half-sweep, just more obvious.
        # Leave those channels alone.
        if n not in CONTROL_CHANNELS and (hi[n] - lo[n]) < CONNECT_FLOOR:
            skipped.append(n)
            continue
        writes = [("RC%d_MIN" % n, lo[n]), ("RC%d_MAX" % n, hi[n])]
        # Throttle trims to its minimum, not its centre: trim is where the
        # stick rests, and a throttle rests down. Everything else trims to
        # where it sat at rest BEFORE the sweep.
        trim = lo[n] if n == 3 else trim_rest.get(n, last.get(n, (lo[n] + hi[n]) // 2))
        writes.append(("RC%d_TRIM" % n, trim))
        for name, value in writes:
            got, readback = link.set_param(master, name, value)
            print("  %-12s -> %-5d %s" % (name, value, "ok" if got else "FAILED"))
            if not got:
                failed.append(name)
    print()
    if skipped:
        print("left alone (never moved, so nothing was measured): %s"
              % ", ".join("ch%d" % n for n in skipped))
    if failed:
        print("%d write(s) did not verify: %s" % (len(failed), ", ".join(failed)))
        return 1
    print("written and verified.")
    if fs_value:
        print()
        print("FS_THR_VALUE %g sits %g us below RC3_MIN %d." % (
            fs_value, lo[3] - fs_value, lo[3]))
        print("Now TEST it: ./rc_live.py --failsafe, switch the transmitter off,")
        print("and confirm ch3 drops below %g. An untested failsafe is the one" % fs_value)
        print("that bites.")
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
