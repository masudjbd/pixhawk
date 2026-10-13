#!/usr/bin/env python3
"""Live RC channels, with travel ranges and a failsafe test.

    ./rc_live.py                 # all channels, live, with min/max seen
    ./rc_live.py --failsafe      # watch what happens when you switch the TX off

Reading this tool:

  An UNCONNECTED input and a live-but-motionless channel look identical in a
  single frame. What separates them is jitter: a real decoded channel wobbles
  a few microseconds frame to frame, an idle default is bit-identical. The
  'jit' column is the point of the display.

  On SBUS every channel arrives in one frame, so either all of them decode or
  none do. A single dead channel among live ones is a transmitter mixing
  problem, not wiring.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

BAR_WIDTH = 28


def bar(value, lo=1000, hi=2000):
    if not value:
        return " " * BAR_WIDTH
    frac = max(0.0, min(1.0, (value - lo) / float(hi - lo)))
    filled = int(round(frac * (BAR_WIDTH - 1)))
    return "." * filled + "#" + "." * (BAR_WIDTH - 1 - filled)


def main():
    ap = argparse.ArgumentParser(description="live RC input")
    ap.add_argument("--port")
    ap.add_argument("--channels", type=int, default=10)
    ap.add_argument("--failsafe", action="store_true",
                    help="failsafe test: watch the throttle channel on signal loss")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)

    fs_value = link.get_param(master, "FS_THR_VALUE")
    rc3_min = link.get_param(master, "RC3_MIN")
    mode_ch = int(link.get_param(master, "FLTMODE_CH") or 5)

    if args.failsafe:
        print()
        print("FAILSAFE TEST. Props off -- this is a bench test, not an air test.")
        print("  FS_THR_VALUE %s, RC3_MIN %s" % (fs_value, rc3_min))
        print()
        print("  1. transmitter ON, throttle at idle -- watch ch3 settle")
        print("  2. switch the transmitter OFF")
        print("  3. ch3 must fall BELOW FS_THR_VALUE for the failsafe to fire")
        print()
        print("What goes wrong: an FS-iA6B that is not configured to output a")
        print("failsafe HOLDS ITS LAST FRAME instead. On the APM 2.8 airframe it")
        print("held 1996 us -- full throttle -- and the board walked into ESC")
        print("calibration on the next boot. If ch3 does not drop below the")
        print("threshold here, fix the receiver's failsafe before flying.")
        print()

    seen_min, seen_max, last = {}, {}, {}
    jitter = {}
    start = time.time()
    print()
    try:
        while True:
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
            if msg is None:
                print("\rno RC_CHANNELS ... is the board streaming?", end="", flush=True)
                continue
            lines = []
            for n in range(1, args.channels + 1):
                value = getattr(msg, "chan%d_raw" % n, 0)
                if value in (0, 65535):
                    lines.append("  ch%-2d  ----    (no signal)" % n)
                    continue
                seen_min[n] = min(seen_min.get(n, value), value)
                seen_max[n] = max(seen_max.get(n, value), value)
                if last.get(n) is not None and last[n] != value:
                    jitter[n] = jitter.get(n, 0) + 1
                last[n] = value
                tag = ""
                if n == 3:
                    if fs_value and value < fs_value:
                        tag = "  <-- BELOW FS_THR_VALUE: failsafe would fire"
                    elif fs_value:
                        tag = "  (fs %g)" % fs_value
                if n == mode_ch:
                    tag += "  <-- mode channel"
                lines.append("  ch%-2d %5d  [%s]  %4d-%-4d  jit %-4d%s" % (
                    n, value, bar(value), seen_min[n], seen_max[n],
                    jitter.get(n, 0), tag))
            out = "\n".join(lines)
            print("\033[H\033[J" if sys.stdout.isatty() else "", end="")
            print("rc_live  %.0fs   rssi %s" % (time.time() - start, msg.rssi))
            print(out)
            print()
            print("  travel of 800+ us on a stick means the endpoints are real;")
            print("  under 200 us is a channel that has barely been moved.")
            print("  Ctrl-C to stop.")
    except KeyboardInterrupt:
        print()
        live = sorted(seen_min)
        if not live:
            print("No channel ever produced a value. Nothing is decoding on RCIN.")
            return 1
        print("summary:")
        for n in live:
            travel = seen_max[n] - seen_min[n]
            note = ""
            if travel >= 800:
                note = "full sweep"
            elif travel >= 200:
                note = "partial -- do not calibrate on this"
            else:
                note = "barely moved"
            print("  ch%-2d  %4d-%-4d  travel %4d us  jit %-5d %s" % (
                n, seen_min[n], seen_max[n], travel, jitter.get(n, 0), note))
        unjittered = [n for n in live if not jitter.get(n)]
        if unjittered:
            print()
            print("  channels that never changed by even 1 us: %s" %
                  ", ".join("ch%d" % n for n in unjittered))
            print("  A decoded channel jitters. These are either untouched or not")
            print("  really being decoded.")
        return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
