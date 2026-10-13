#!/usr/bin/env python3
"""Check which WAY each stick moves the aircraft, not just how far.

    ./check_stick_sense.py

The fault this exists to catch is invisible to everything else. A reversed
channel and a correct one produce identical PWM ranges, so a parameter dump,
a radio calibration and `rc_live.py` all look perfect. The aircraft still
self-levels, because that comes from the IMU, so it hovers beautifully -- and
then flies backwards when you push the stick forward.

Only the SIGN of the stick's movement relative to its trim separates them,
which means somebody has to move each stick in a named direction and have a
tool watch. That is this.

Measured on the APM 2.8 airframe with this radio: the FS-i6's elevator output
ran opposite to what ArduPilot expects -- pitch stick forward gave a POSITIVE
pitch demand, i.e. nose up, i.e. fly backwards. Fixed with RC2_REV -1 (the
parameter is RC2_REVERSED on 4.x). Roll measured correct in the same run,
which is what ruled out a board mounted 180 degrees round -- that reverses
BOTH axes and is fixed with AHRS_ORIENTATION, never by reversing channels.

Re-measure on new hardware. Do not assume it carried over.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

# (name, RCMAP parameter, prompt, the sign ArduPilot expects for that move)
#
# ArduPilot's convention, for a stick pushed in the named direction:
#   roll  right  -> positive  (rolls right, flies right)
#   pitch forward-> NEGATIVE  (nose down, flies forward)
#   yaw   right  -> positive  (nose right)
#   throttle up  -> positive
TESTS = [
    ("roll", "RCMAP_ROLL", "push the ROLL stick fully RIGHT", +1,
     "rolls right and flies right"),
    ("pitch", "RCMAP_PITCH", "push the PITCH stick fully FORWARD (away from you)", -1,
     "nose down and flies forward"),
    ("yaw", "RCMAP_YAW", "push the YAW stick fully RIGHT", +1,
     "nose yaws right"),
    ("throttle", "RCMAP_THROTTLE", "push the THROTTLE stick fully UP", +1,
     "climbs"),
]


def capture_mode(master, seconds):
    """Measure all four axes from one two-stick gesture, with no prompts.

    The per-axis prompts exist so a human can be told what to do. When the
    tool is driven from somewhere the operator cannot see, that breaks -- so
    this mode takes the PEAK deviation from trim on every channel over one
    window instead. Peaks do not care about ordering or timing: hold both
    sticks into their corners at any point and all four signs are captured.

    Right stick to FORWARD-RIGHT gives roll-right and pitch-forward.
    Left stick to UP-RIGHT gives throttle-up and yaw-right.
    """
    channels, trims, revs = {}, {}, {}
    for name, rcmap_param, _, expected, consequence in TESTS:
        ch = int(link.get_param(master, rcmap_param) or 0)
        if not ch:
            continue
        channels[name] = (ch, expected, consequence)
        trims[ch] = link.get_param(master, "RC%d_TRIM" % ch)
        rev = link.get_param(master, "RC%d_REVERSED" % ch)
        if rev is None:
            rev = link.get_param(master, "RC%d_REV" % ch)
        revs[ch] = rev

    print()
    print("Capturing for %.0fs. Hold the RIGHT stick to the FORWARD-RIGHT corner" % seconds)
    print("and the LEFT stick to the UP-RIGHT corner, together, at any point.")
    print()

    # Track BOTH extremes per channel, not one signed peak. A channel that
    # was swept through its whole range reaches both, and the larger of the
    # two is then an accident of where the trim sits -- it carries no
    # information about which way the stick was pushed. Recording only the
    # peak magnitude produced a confident, wrong "both axes reversed" verdict
    # on this airframe from a full sweep. Measure first whether the
    # measurement can support a conclusion.
    pos, neg = {}, {}
    samples = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
        if msg is None:
            continue
        samples += 1
        for ch, trim in trims.items():
            value = getattr(msg, "chan%d_raw" % ch, 0)
            if value in (0, 65535):
                continue
            delta = value - (trim or 1500)
            if delta > 0:
                pos[ch] = max(pos.get(ch, 0), delta)
            else:
                neg[ch] = min(neg.get(ch, 0), delta)

    print("%d samples" % samples)
    if not samples:
        print("no RC data at all.")
        return 1

    MOVED = 150
    print()
    print("  axis      ch  trim    down/left  up/right   demand  expected  verdict")
    wrong, untested, ambiguous = [], [], []
    for name, (ch, expected, consequence) in channels.items():
        up, down = pos.get(ch, 0), neg.get(ch, 0)
        if abs(up) < MOVED and abs(down) < MOVED:
            print("  %-9s %-3d %-7s %+9d %+9d %7s %9d  NOT MOVED -- says nothing"
                  % (name, ch, trims.get(ch), down, up, "--", expected))
            untested.append(name)
            continue
        if abs(up) >= MOVED and abs(down) >= MOVED:
            # Both extremes reached: the stick was swept, so there is no
            # "the direction you pushed" to read a sign from.
            print("  %-9s %-3d %-7s %+9d %+9d %7s %9d  SWEPT -- cannot tell"
                  % (name, ch, trims.get(ch), down, up, "--", expected))
            ambiguous.append(name)
            continue
        delta = up if abs(up) >= MOVED else down
        raw_sign = 1 if delta > 0 else -1
        effective = -raw_sign if revs.get(ch) in (1, -1) and revs.get(ch) != 0 else raw_sign
        verdict = "correct -- %s" % consequence if effective == expected else "REVERSED"
        if effective != expected:
            wrong.append((name, ch, revs.get(ch)))
        print("  %-9s %-3d %-7s %+9d %+9d %7d %9d  %s"
              % (name, ch, trims.get(ch), down, up, effective, expected, verdict))

    print()
    if ambiguous:
        print("SWEPT, so no conclusion: %s" % ", ".join(ambiguous))
        print("Those channels reached BOTH extremes, which means the stick was moved")
        print("back and forth. The sign of a peak is then just an accident of where")
        print("the trim sits. Re-run and HOLD ONE CORNER without sweeping:")
        print("  right stick -> FORWARD-RIGHT, left stick -> UP-RIGHT, hold still.")
        return 1
    if untested:
        print("Not exercised: %s. Re-run and push those sticks fully into the"
              % ", ".join(untested))
        print("named corners -- a channel that did not move proves nothing either way.")
    if not wrong and not untested:
        print("All four axes move the aircraft the way the stick says.")
        return 0
    if len(wrong) >= 2 and {w[0] for w in wrong} >= {"roll", "pitch"}:
        print()
        print("BOTH roll and pitch are reversed. That is a board mounted 180 degrees")
        print("round, NOT two reversed channels -- fix it with AHRS_ORIENTATION.")
        print("Reversing two channels would make the sticks feel right while leaving")
        print("the estimator's idea of forward wrong, breaking every autonomous mode.")
        return 1
    for name, ch, rev in wrong:
        print("  %s (channel %d) is REVERSED. Fix with RC%d_REVERSED 1, or reverse"
              % (name, ch, ch))
        print("    that channel in the FS-i6's Reverse menu. One or the other, never both.")
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description="stick direction sense check")
    ap.add_argument("--port")
    ap.add_argument("--window", type=float, default=4.0)
    ap.add_argument("--capture", type=float, metavar="SECONDS",
                    help="prompt-free: sample for this long and take the peak "
                         "deviation from trim on all four axes at once. Hold the "
                         "right stick to the FORWARD-RIGHT corner and the left "
                         "stick to the UP-RIGHT corner together at any point "
                         "during the window -- peaks do not care when.")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=20)

    if args.capture:
        return capture_mode(master, args.capture)

    print()
    print("Transmitter ON, props OFF, aircraft on the bench.")
    print("Each stick in turn: hold it at the named extreme for the 4s window.")
    print()

    results = []
    for name, rcmap_param, prompt, expected, consequence in TESTS:
        channel = int(link.get_param(master, rcmap_param) or 0)
        if not channel:
            print("  %s: could not read %s -- skipping" % (name, rcmap_param))
            continue
        trim = link.get_param(master, "RC%d_TRIM" % channel)
        reversed_param = "RC%d_REVERSED" % channel
        rev = link.get_param(master, reversed_param)
        if rev is None:
            reversed_param = "RC%d_REV" % channel
            rev = link.get_param(master, reversed_param)

        print("  %s -> channel %d, trim %s, %s %s" % (
            name.upper(), channel, trim, reversed_param, rev))
        if trim in (None, 0):
            print("    no trim stored: run ./calibrate_radio.py --write first, or the")
            print("    sign below is measured against nothing.")
        input("    %s, then press Return and HOLD it: " % prompt)

        peak, deadline = None, time.time() + args.window
        while time.time() < deadline:
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
            if msg is None:
                continue
            value = getattr(msg, "chan%d_raw" % channel, 0)
            if not value:
                continue
            delta = value - (trim or 1500)
            if peak is None or abs(delta) > abs(peak):
                peak = delta
        if peak is None:
            print("    no RC data for channel %d." % channel)
            results.append((name, channel, None, expected, rev, consequence))
            continue

        # The channel's raw sign, then what the firmware does with it.
        raw_sign = 1 if peak > 0 else -1
        effective = raw_sign * (-1 if rev in (-1, 1) and rev == -1 else 1)
        if rev == 1 and reversed_param.endswith("REVERSED"):
            effective = -raw_sign          # RCn_REVERSED is 0/1, not -1/+1
        print("    stick moved the channel by %+d us -> demand sign %+d (expected %+d)"
              % (peak, effective, expected))
        results.append((name, channel, effective, expected, rev, consequence))
        print()

    print("summary")
    print("  axis      ch  measured  expected  verdict")
    wrong = []
    for name, channel, effective, expected, rev, consequence in results:
        if effective is None:
            verdict = "no data"
        elif effective == expected:
            verdict = "correct -- %s" % consequence
        else:
            verdict = "REVERSED"
            wrong.append((name, channel, rev))
        print("  %-9s %-3d %8s %9d  %s" % (
            name, channel, "%+d" % effective if effective else "--", expected, verdict))

    print()
    if not wrong:
        print("All four axes move the aircraft the way the stick says.")
        return 0

    if len(wrong) >= 2 and {w[0] for w in wrong} >= {"roll", "pitch"}:
        print("BOTH roll and pitch are reversed. That is a board mounted 180 degrees")
        print("round relative to the airframe, NOT two reversed channels. Fix it with")
        print("AHRS_ORIENTATION (or by turning the board), and then re-run. Reversing")
        print("two channels would make the sticks feel right while leaving the")
        print("estimator's idea of forward wrong, which breaks every autonomous mode.")
        return 1

    for name, channel, rev in wrong:
        print("  %s (channel %d) is reversed." % (name, channel))
        print("    Fix on the board:  RC%d_REVERSED 1" % channel)
        print("    or on the radio:   reverse that channel in the FS-i6's Reverse menu")
        print("    Do one or the other, never both.")
    return 1


if __name__ == "__main__":
    sys.exit(link.run(main))
