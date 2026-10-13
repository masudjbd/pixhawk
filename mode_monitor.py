#!/usr/bin/env python3
"""Live: mode-switch PWM, the slot it selects, and the flight mode that results.

    ./mode_monitor.py

Flick the switch through every position and watch all three columns agree.
Three things this catches that a parameter dump cannot:

  * a 2-position switch assigned to the mode channel. On the FS-i6 only SwC
    is 3-position -- SwA, SwB and SwD are not -- so a mode set that assumes
    three reachable slots silently gives you two.
  * slot boundaries that do not land where the profile assumed. Transmitter
    endpoint and subtrim settings move the actual PWM, and ArduCopter's
    readSwitch() boundaries are fixed, so your switch may select 1/3/5
    instead of 1/4/6.
  * a mode that is configured but not available -- the board refuses LOITER
    without a position estimate, and says so in STATUSTEXT.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

# ArduCopter's readSwitch() boundaries, in microseconds. Fixed in firmware.
SLOT_BOUNDS = [(0, 1230, 1), (1231, 1360, 2), (1361, 1490, 3),
               (1491, 1620, 4), (1621, 1749, 5), (1750, 3000, 6)]

MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
         5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
         13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
         18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
         22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID",
         26: "AUTOROTATE", 27: "AUTO_RTL"}


def slot_for(pwm):
    for lo, hi, slot in SLOT_BOUNDS:
        if lo <= pwm <= hi:
            return slot
    return None


def main():
    ap = argparse.ArgumentParser(description="live flight-mode switch monitor")
    ap.add_argument("--port")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=10)

    ch = int(link.get_param(master, "FLTMODE_CH") or 5)
    configured = {}
    for slot in range(1, 7):
        value = link.get_param(master, "FLTMODE%d" % slot)
        configured[slot] = int(value) if value is not None else None

    print()
    print("mode channel: ch%d" % ch)
    print("configured slots:")
    for slot in range(1, 7):
        mode = configured[slot]
        print("  slot %d  %-12s (%s)" % (
            slot, MODES.get(mode, "?"), mode if mode is not None else "--"))
    print()
    print("Flick the switch through every position. Ctrl-C when done.")
    print()
    print("  ch%d pwm   slot   configured      board reports" % ch)

    seen_slots, seen_modes = set(), set()
    last_line = None
    try:
        while True:
            msg = master.recv_match(type=["RC_CHANNELS", "HEARTBEAT", "STATUSTEXT"],
                                    blocking=True, timeout=2)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                text = link.statustext(msg)
                if any(k in text for k in ("Mode", "mode", "requires", "Flight")):
                    print("  note: %s" % text)
                continue
            if kind == "HEARTBEAT":
                main.current = MODES.get(msg.custom_mode, str(msg.custom_mode))
                seen_modes.add(main.current)
                continue
            pwm = getattr(msg, "chan%d_raw" % ch, 0)
            if not pwm:
                line = "  ----      --     --              (no RC input on ch%d)" % ch
            else:
                slot = slot_for(pwm)
                seen_slots.add(slot)
                mode = configured.get(slot)
                line = "  %4d     %4s   %-14s  %s" % (
                    pwm, slot, MODES.get(mode, "?"),
                    getattr(main, "current", "?"))
            if line != last_line:
                print(line)
                last_line = line
    except KeyboardInterrupt:
        print()
        reached = sorted(s for s in seen_slots if s)
        print("slots reached: %s" % (", ".join(str(s) for s in reached) or "none"))
        if len(reached) < 2:
            print()
            print("The switch only ever selected one slot, so this run says nothing")
            print("about what it can reach -- it does NOT mean the switch is broken.")
            print("Flick it through every detent and run again.")
        elif len(reached) == 2:
            print()
            print("Only two slots. That is a 2-POSITION switch on ch%d. On the FS-i6," % ch)
            print("SwC is the only 3-position switch: assign it in Functions setup ->")
            print("Aux. channels -> Channel %d = SwC, and save with a LONG press of" % ch)
            print("CANCEL (a short press discards, which is why it 'does not stick').")
        else:
            print()
            print("Three or more slots reachable -- that is what the 3-position")
            print("profiles assume. Make sure the modes you want are on the slots")
            print("you can actually reach: %s" % ", ".join(
                "slot %d=%s" % (s, MODES.get(configured.get(s), "?")) for s in reached))
        print("modes the board reported: %s" % ", ".join(sorted(seen_modes)) or "none")
        return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
