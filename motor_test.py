#!/usr/bin/env python3
"""Spin each motor on its own, to prove wiring order and rotation direction.

    ./motor_test.py --props-off --seq 1          # one motor
    ./motor_test.py --props-off --all            # 1,2,3,4 in test order
    ./motor_test.py --props-off --seq 2 --throttle 20 --time 3

THIS COMMANDS REAL MOTOR OUTPUT WITH A LIVE BATTERY. Props off the motors --
not "pointed away", off. The --props-off flag is required, so that spinning a
motor is never one typo away.

The single most misread thing about this command: param1 of
MAV_CMD_DO_MOTOR_TEST is the TEST SEQUENCE NUMBER, NOT the motor number.
ArduPilot walks clockwise round the airframe starting at the front-right arm.
For a quad X:

    sequence 1 -> Motor 1, front-right, spins CCW
    sequence 2 -> Motor 4, rear-right,  spins CW
    sequence 3 -> Motor 2, rear-left,   spins CCW
    sequence 4 -> Motor 3, front-left,  spins CW

Expect sequence 2 to spin the REAR-RIGHT motor. Reading it as "motor 2" makes
a correctly wired aircraft look scrambled, and leads people to swap leads
that were right to begin with.

Three things that each look like a fault and are not:

  Nothing spinning while every command is ACCEPTED. The first suspect is the
  throttle being below the ESCs' start threshold, not the wiring -- confirm
  which it is by watching SERVO_OUTPUT_RAW during the test: if the commanded
  channel really does go to the requested microseconds, the board has done
  its job and the fault is downstream of the pin.

  "Motor Test: vehicle not landed" for the first ~15 s after boot. The land
  detector has to settle, and opening the serial port can reboot the board,
  so every invocation waits before commanding.

  8% throttle can sit below the ESC's start threshold: the command is
  ACCEPTED and nothing turns. 15% is the default here for that reason.

  Battery reading 0.00 V for the first few seconds, which is
  indistinguishable from an unplugged pack. Settle first, then read.

## Percent throttle needs a calibrated radio; PWM does not

ArduCopter's motor-test checks differ by throttle type. Copter calls

    mavlink_motor_test_check(chan, throttle_type != 1, motor_count)

so the RC-calibration and throttle-at-zero checks run for PERCENT throttle
(type 0) and are SKIPPED for direct PWM (type 1) -- the comment upstream says
the RC check "can be skipped if direct pwm is supplied".

On an aircraft whose receiver is not wired yet, a percent test is therefore
refused with "Motor Test: RC not calibrated" while a PWM test runs fine. That
is the whole reason --pwm exists:

    ./motor_test.py --props-off --all --pwm 1150

1250 on the default MOT_PWM_MIN/MAX of 1000/2000 is about 25 percent. Start
there, not lower: on this airframe's uncalibrated SimonK/BLHeli ESCs, 1150
started NONE of the four while the board was demonstrably driving the pin at
exactly 1150 us -- an accepted command and a silent motor that had nothing to
do with wiring. Do not go near 1500 on a bench.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

QUAD_X = {1: (1, "front-right", "CCW"), 2: (4, "rear-right", "CW"),
          3: (2, "rear-left", "CCW"), 4: (3, "front-left", "CW")}

ACK_RESULTS = {0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED",
               3: "UNSUPPORTED", 4: "FAILED", 5: "IN PROGRESS", 6: "CANCELLED"}


def p_or(value, fallback):
    return fallback if value in (None, 0) else value


def main():
    ap = argparse.ArgumentParser(description="per-motor test")
    ap.add_argument("--port")
    ap.add_argument("--props-off", action="store_true",
                    help="required: confirms the propellers are off the motors")
    ap.add_argument("--seq", type=int, action="append", default=[],
                    help="test sequence number (1-4 on a quad), repeatable")
    ap.add_argument("--all", action="store_true", help="sequences 1..4 in order")
    ap.add_argument("--throttle", type=float, default=15.0, help="percent (default 15)")
    ap.add_argument("--pwm", type=int,
                    help="send raw PWM microseconds instead of a percentage. Skips "
                         "ArduCopter's RC-calibration check, so this is the option "
                         "to use before the receiver is wired. Try 1250.")
    ap.add_argument("--time", type=float, default=2.0, help="seconds per motor")
    ap.add_argument("--settle", type=float, default=16.0,
                    help="seconds to wait before commanding (land detector)")
    ap.add_argument("--no-battery-check", action="store_true")
    ap.add_argument("--yes", "-y", action="store_true",
                    help="do not prompt between motors. Announces each motor with a "
                         "countdown and reports the acks; you report what actually "
                         "turned afterwards. For driving this from a session that has "
                         "no keyboard at the aircraft.")
    args = ap.parse_args()

    if not args.props_off:
        print("Refusing: pass --props-off.", file=sys.stderr)
        print(file=sys.stderr)
        print("This spins motors for real. Take the propellers OFF the motors --", file=sys.stderr)
        print("not pointed away, off -- then pass the flag.", file=sys.stderr)
        return 2
    sequences = sorted(set(args.seq)) or ([1, 2, 3, 4] if args.all else [])
    if not sequences:
        print("nothing to do: pass --seq N or --all.", file=sys.stderr)
        return 2
    if args.throttle > 40:
        print("Refusing a throttle above 40%% on a bench test.", file=sys.stderr)
        return 2
    if args.pwm is not None and not 1000 <= args.pwm <= 1400:
        print("Refusing --pwm %d: stay between 1000 and 1400 on a bench test."
              % args.pwm, file=sys.stderr)
        print("1150 is about 15%% of the default 1000-2000 range.", file=sys.stderr)
        return 2

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=5)
    sys_id, comp = link.target(master)
    # SERVO_OUTPUT_RAW fast enough to catch a two-second pulse. Measuring the
    # pin is what separates "the board did nothing" from "the board did its
    # job and the ESC ignored it" -- the single most useful thing this tool
    # can tell you when a motor stays silent.
    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        link.mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20000, 0, 0, 0, 0, 0)

    fclass = link.get_param(master, "FRAME_CLASS")
    ftype = link.get_param(master, "FRAME_TYPE")
    print()
    print("frame: %s %s" % (link.FRAME_CLASSES.get(int(fclass or 0), fclass),
                            link.FRAME_TYPES.get(int(ftype or 0), ftype)))
    if not fclass:
        print()
        print("FRAME_CLASS is 0: there is no mixer, so no output is a motor and")
        print("this test cannot do anything. Set the frame first.")
        return 1

    # Choose the throttle mode from what this aircraft can actually do, rather
    # than defaulting to percent and letting the firmware reject it.
    #
    # ArduCopter checks RC calibration for PERCENT throttle and skips the check
    # for direct PWM. On an aircraft with no receiver, a percent test is
    # therefore refused -- and the refusal is easy to miss in a wall of
    # output, which looks exactly like four dead ESCs. So: detect it, say so,
    # and switch.
    pwm_min = p_or(link.get_param(master, "MOT_PWM_MIN"), 1000)
    pwm_max = p_or(link.get_param(master, "MOT_PWM_MAX"), 2000)
    rc1_min = link.get_param(master, "RC1_MIN")
    rc1_max = link.get_param(master, "RC1_MAX")
    rc_calibrated = not (rc1_min in (None, 1100, 1000) and rc1_max in (None, 1900, 2000))

    rc_live = False
    deadline = time.time() + 3
    while time.time() < deadline and not rc_live:
        msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
        if msg is not None:
            rc_live = any(getattr(msg, "chan%d_raw" % n, 0) not in (0, 65535)
                          for n in range(1, 5))

    if args.pwm is None and not (rc_calibrated and rc_live):
        args.pwm = 1250
        print()
        print("No usable RC input (%s%s), so ArduCopter would REFUSE a percent-"
              % ("radio not calibrated" if not rc_calibrated else "radio calibrated",
                 ", no live channels" if not rc_live else ""))
        print("throttle motor test with 'RC not calibrated'. Switching to direct")
        print("PWM at %d us, which skips that check." % args.pwm)
    elif args.pwm is None:
        mapped = pwm_min + (args.throttle / 100.0) * (pwm_max - pwm_min)
        print()
        print("percent throttle %.0f%% maps to about %.0f us on MOT_PWM_MIN/MAX "
              "%g/%g" % (args.throttle, mapped, pwm_min, pwm_max))
        if mapped < 1200:
            print("  WARNING: that is low. On this airframe's uncalibrated ESCs,")
            print("  1150 us started none of the four. Consider --pwm 1250.")

    print()
    print("waiting %.0fs for the land detector to settle" % args.settle)
    print("(a fresh connection can reboot the board, and ArduPilot refuses a")
    print(" motor test until it is sure the aircraft is on the ground)")
    deadline = time.time() + args.settle
    volts = 0.0
    while time.time() < deadline:
        msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
        if msg is not None:
            volts = msg.voltage_battery / 1000.0
        left = deadline - time.time()
        print("\r  %4.0fs   battery %.2f V   " % (left, volts), end="", flush=True)
    print()

    if volts < 6.0 and not args.no_battery_check:
        print()
        print("battery reads %.2f V -- the pack looks unplugged." % volts)
        print("Motors cannot turn without it, and MAIN OUT takes no power from")
        print("USB. Plug the pack in (props OFF), or pass --no-battery-check to")
        print("send the commands anyway and watch the ACKs.")
        return 1

    # The safety switch is the one failure that looks exactly like success:
    # the command is ACCEPTED, the ack says so, and no output moves. Check it
    # rather than letting the user conclude four ESCs are dead.
    status = None
    deadline = time.time() + 4
    while time.time() < deadline and status is None:
        status = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
    if status is not None and not status.onboard_control_sensors_enabled & 0x8000:
        print()
        print("SAFETY SWITCH IS ENGAGED -- outputs are inhibited.")
        print()
        print("The motor test would be ACCEPTED and nothing would turn, which")
        print("looks identical to four dead ESCs. Press and hold the safety")
        print("button until its LED stops flashing, then run this again.")
        print()
        print("Props off before you do that: disengaging the switch makes the")
        print("outputs live.")
        return 1
    print()
    print("safety switch is off -- outputs are LIVE.")
    print()

    results = []
    for seq in sequences:
        motor, where, spin = QUAD_X.get(seq, (seq, "?", "?"))
        print("sequence %d -> Motor %d, %s, should spin %s" % (seq, motor, where, spin))
        if args.pwm is not None:
            how = "%d us" % args.pwm
            throttle_type, throttle_value = 1, args.pwm
        else:
            how = "%.0f%%" % args.throttle
            throttle_type, throttle_value = 0, args.throttle
        if args.yes:
            print("  WATCH THE %s MOTOR. Spinning for %.1fs at %s in"
                  % (where.upper(), args.time, how), end="", flush=True)
            for count in (3, 2, 1):
                print(" %d" % count, end="", flush=True)
                time.sleep(1.0)
            print(" -- now")
        else:
            input("  Return to spin it for %.1fs at %s: " % (args.time, how))
        master.mav.command_long_send(
            sys_id, comp, link.mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            seq,                # test sequence number
            throttle_type,      # 0 = percent, 1 = raw PWM
            throttle_value,
            args.time,
            0,                  # motor count: 0 = just this one
            0, 0)
        ack, texts, peak = None, [], {}
        end = time.time() + args.time + 3
        while time.time() < end:
            msg = master.recv_match(
                type=["COMMAND_ACK", "STATUSTEXT", "SERVO_OUTPUT_RAW"],
                blocking=True, timeout=0.5)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "SERVO_OUTPUT_RAW":
                for c in range(1, 9):
                    value = getattr(msg, "servo%d_raw" % c, 0)
                    peak[c] = max(peak.get(c, 0), value)
                continue
            if kind == "COMMAND_ACK":
                if msg.command == link.mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
                    ack = msg.result
            else:
                text = link.statustext(msg)
                if text:
                    texts.append(text)
                    print("    board: %s" % text)
        print("    ack: %s" % ACK_RESULTS.get(ack, ack))

        # What the pin actually did. Output n drives Motor n, so the channel
        # to expect is the motor number, not the sequence number.
        driven = peak.get(motor, 0)
        others = [c for c, v in peak.items() if c != motor and v > pwm_min + 20]
        if peak:
            print("    output MAIN %d peaked at %d us%s" % (
                motor, driven,
                "  (other channels moved: %s)" % ", ".join("MAIN %d" % c for c in others)
                if others else ""))
            if driven <= pwm_min + 20:
                print("    the board did NOT raise that output. Not an ESC problem:")
                print("    check the safety switch and FRAME_CLASS.")
            elif ack == 0:
                print("    the board DID drive the pin. If nothing turned, the fault is")
                print("    downstream of it: ESC power, the signal lead, the motor")
                print("    phases -- or a throttle below that ESC's start threshold.")
        if ack == 0 and args.yes:
            answer = "?"
        elif ack == 0:
            answer = input("    Did the %s motor turn, and in the %s direction? [y/n/?] "
                           % (where, spin)).strip().lower()
        else:
            answer = "n"
            if ack == 1:
                print("    TEMPORARILY REJECTED usually means the land detector is")
                print("    still unhappy, or the safety switch is engaged.")
            if any("RC not calibrated" in t for t in texts):
                print("    'RC not calibrated' blocks PERCENT throttle only. Re-run")
                print("    with --pwm 1150, which uses direct PWM and skips that")
                print("    check -- the right option until the receiver is wired.")
            if any("throttle" in t.lower() and "zero" in t.lower() for t in texts):
                print("    'throttle must be zero' is also a percent-throttle check.")
                print("    --pwm 1150 skips it.")
        results.append((seq, motor, where, spin, ack, answer))
        print()

    print("summary")
    print("  seq  motor  position      expected  ack            answer")
    for seq, motor, where, spin, ack, answer in results:
        print("  %-4d %-6d %-13s %-9s %-14s %s" % (
            seq, motor, where, spin, ACK_RESULTS.get(ack, ack), answer))

    if args.yes:
        print()
        print("Commands were sent and acknowledged as above. What this run CANNOT")
        print("tell you is which motor actually turned and which way -- that needs")
        print("eyes on the aircraft. For each sequence, note:")
        print("  - did a motor turn at all?")
        print("  - was it the one named above?")
        print("  - did it turn in the direction named above?")
        accepted = [r for r in results if r[4] == 0]
        if len(accepted) == len(results):
            print()
            print("All %d commands were ACCEPTED, so the mixer and the link are fine." % len(results))
            print("Anything that did not move is below the ESC start threshold, or")
            print("that ESC has no signal -- not a rejected command.")
        return 0

    wrong = [r for r in results if r[5] == "n"]
    print()
    if not wrong:
        print("All motors turned where and how they should.")
        print("Wiring order and rotation are correct.")
        return 0
    print("What the failures mean:")
    print("  WRONG MOTOR TURNED -> two ESC signal leads are swapped on the")
    print("    board's MAIN OUT header. Swap the two leads; do not renumber")
    print("    anything in software.")
    print("  RIGHT MOTOR, WRONG DIRECTION -> swap any two of the three motor")
    print("    phases at that ESC, or reverse it in BLHeli if it is a BLHeli ESC.")
    print("  NOTHING TURNED, ack ACCEPTED -> throttle below the ESC's start")
    print("    threshold (try --throttle 20), the safety switch engaged, or that")
    print("    ESC has no signal.")
    return 1


if __name__ == "__main__":
    sys.exit(link.run(main, "interrupted. Motors stop on their own when the test timeout expires."))
