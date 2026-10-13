#!/usr/bin/env python3
"""Run ArduPilot's own six-position accelerometer calibration.

    ./calibrate_accel.py

Unlike the APM 2.8 toolkit, nothing is computed here: 4.x carries the
calibration routine on board and this tool drives the handshake. The board
asks for a position over STATUSTEXT, we answer with
MAV_CMD_ACCELCAL_VEHICLE_POS, and it does the fitting.

Order matters for what comes after: the compass fit is interpreted in the
body frame the accelerometers define, so accel first, compass second. And
./level_calibrate.py after both -- it sets the level reference, which is a
different thing from the accel calibration and gets confused for it.

The six positions, as the firmware names them. "Level" means the aircraft
sitting normally on a flat surface; the rest are that surface rotated:

    level      normal attitude, on a flat table
    left       left side down
    right      right side down
    nose down  nose pointing at the floor
    nose up    nose pointing at the ceiling
    back       upside down

Hold each one still until the board acknowledges it. A flat reference
surface matters more than precision: a table that is 2 degrees off makes
every position 2 degrees off, and that becomes the aircraft's idea of level.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

# MAV_CMD_ACCELCAL_VEHICLE_POS values.
POSITIONS = [(1, "level"), (2, "left"), (3, "right"),
             (4, "nose down"), (5, "nose up"), (6, "back")]

# Which body axis gravity lands on for each position, and which earlier
# position it must be the OPPOSITE of.
#
# Deliberately not hard-coding the sign conventions: "level" is whatever the
# board reads when it is sitting normally, and every other position is
# defined relative to that. Pinning absolute signs would bake in an
# assumption about ArduPilot's body frame that is easy to get backwards and
# produces a calibration that completes and is wrong.
POSITION_AXIS = {1: "z", 2: "y", 3: "y", 4: "x", 5: "x", 6: "z"}
POSITION_OPPOSES = {3: 2, 5: 4, 6: 1}

# Buzzer cues, verified audible on this airframe before being relied on.
# Each has a distinct shape so it cannot be mistaken for another: the
# operator is holding the aircraft and cannot see any output.
TUNE_START = b"MFT200L2c"            # one long low note: start holding
TUNE_WARN = b"MFT200L16e"            # short tick: 5 seconds left
TUNE_MOVING = b"MFT200L4caca"        # two-tone alarm: still moving
TUNE_BANKED = b"MFT200L16cdefg"      # rising run: banked, move to the next
TUNE_DONE = b"MFT200L8ceg>c<gec"     # melody: all six done

DOMINANT_MIN = 700      # mG: gravity must clearly be on one axis
STEADY_BAND = 70        # mG: peak-to-peak allowed while "still"
POS_BY_WORD = {
    "level": 1, "left": 2, "right": 3,
    "down": 4, "nosedown": 4, "up": 5, "noseup": 5, "back": 6,
}


def beep(master, tune):
    """Play a tune on the board's buzzer, if one is fitted.

    The whole difficulty of driving this remotely is that the person holding
    the aircraft cannot see the tool's output. A buzzer gives them a channel
    that does not need a screen: one note when a position is banked, a run of
    notes when the whole calibration lands.
    """
    try:
        sys_id, comp = link.target(master)
        master.mav.play_tune_send(sys_id, comp, tune)
    except Exception:
        pass


def hold_for_position(master, position, name, signs, seconds, settle):
    """Give the operator a fixed window, then accept the position.

    Deliberately NOT adaptive: a window that ends early drifts the schedule
    out from under someone following a clock, and losing sync is worse than
    waiting a few extra seconds. Being early is harmless -- they just hold
    the position longer.
    """
    axis = POSITION_AXIS[position]
    index = {"x": 0, "y": 1, "z": 2}[axis]
    deadline = time.time() + seconds
    window, last_report, warned = [], 0.0, False

    while time.time() < deadline:
        reading = read_accel(master)
        if reading is None:
            continue
        window.append((time.time(), reading))
        window[:] = [w for w in window if w[0] > time.time() - settle]
        remaining = deadline - time.time()
        # A single tick five seconds out: the only cue the operator has that
        # the window is about to close, since they cannot see this output.
        if remaining <= 5.0 and not warned:
            warned = True
            beep(master, TUNE_WARN)
            print("    5s left -- hold it still")
        if time.time() - last_report > 5.0:
            last_report = time.time()
            print("    %4.0fs left   x%+5d y%+5d z%+5d"
                  % (remaining, reading[0], reading[1], reading[2]))

    # Window is up. Insist on a still reading before banking it -- a sample
    # taken mid-move is the main way a six-position calibration completes and
    # is quietly wrong. Wait past the window if it is still moving.
    grace = time.time() + 20
    while time.time() < grace:
        reading = read_accel(master)
        if reading is None:
            continue
        window.append((time.time(), reading))
        window[:] = [w for w in window if w[0] > time.time() - settle]
        if len(window) < 4:
            continue
        spans = [max(w[1][i] for w in window) - min(w[1][i] for w in window)
                 for i in range(3)]
        if max(spans) <= STEADY_BAND:
            value = reading[index]
            signs[position] = 1 if value > 0 else -1
            on_axis = abs(value) > DOMINANT_MIN
            print("    banked: %s axis %+d, steady%s"
                  % (axis.upper(), value,
                     "" if on_axis else "  <-- WARNING: gravity is not on %s, "
                                        "was the aircraft in the right position?"
                                        % axis.upper()))
            return True
        print("    still moving (%d mG) -- hold it still" % max(spans))
        beep(master, TUNE_MOVING)
    print("    never settled; banking anyway")
    return True


def read_accel(master, timeout=1.0):
    """Latest (x, y, z) accelerometer reading in mG, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        link.heartbeat(master)
        msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=0.3)
        if msg is not None:
            return (msg.xacc, msg.yacc, msg.zacc)
    return None


def wait_for_position(master, position, name, signs, settle, timeout):
    """Block until the aircraft is held steady in `position`. Returns True/False.

    Three things must hold before a position is accepted:
      * gravity is clearly on the expected axis (over DOMINANT_MIN)
      * the reading is STILL -- peak-to-peak under STEADY_BAND across the
        settle window. A sample taken mid-move is the main way a six-position
        calibration completes and is wrong.
      * where the position is the opposite of an earlier one (right vs left),
        the sign actually differs, so holding the same side twice cannot be
        mistaken for progress.
    """
    axis = POSITION_AXIS[position]
    index = {"x": 0, "y": 1, "z": 2}[axis]
    opposes = POSITION_OPPOSES.get(position)
    want_opposite_of = signs.get(opposes) if opposes else None

    window = []
    deadline = time.time() + timeout
    last_report = 0.0
    while time.time() < deadline:
        reading = read_accel(master)
        if reading is None:
            continue
        window.append((time.time(), reading))
        window[:] = [w for w in window if w[0] > time.time() - settle]

        value = reading[index]
        others = [abs(v) for i, v in enumerate(reading) if i != index]
        dominant = abs(value) > DOMINANT_MIN and abs(value) > max(others) * 1.5
        sign = 1 if value > 0 else -1
        sign_ok = want_opposite_of is None or sign == -want_opposite_of

        steady = False
        if len(window) >= 4 and window[0][0] <= time.time() - settle * 0.9:
            for i in range(3):
                values = [w[1][i] for w in window]
                if max(values) - min(values) > STEADY_BAND:
                    break
            else:
                steady = True

        if time.time() - last_report > 2.0:
            last_report = time.time()
            state = []
            if not dominant:
                state.append("gravity not on %s yet" % axis.upper())
            elif not sign_ok:
                state.append("that is the same side as the previous position")
            if not steady:
                state.append("still moving")
            print("    x%+5d y%+5d z%+5d   %s" % (
                reading[0], reading[1], reading[2],
                ", ".join(state) or "holding steady..."))

        if dominant and sign_ok and steady:
            signs[position] = sign
            print("    accepted: %s axis %+d, steady" % (axis.upper(), value))
            return True
    print("    TIMED OUT waiting for '%s'" % name)
    return False


def main():
    ap = argparse.ArgumentParser(description="six-position accel calibration")
    ap.add_argument("--port")
    ap.add_argument("--auto", action="store_true",
                    help="no prompts: watch the accelerometer and acknowledge each "
                         "position once the aircraft is held steady in it. Use when "
                         "the operator cannot see this tool's output.")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds the reading must stay steady before accepting")
    ap.add_argument("--position-timeout", type=float, default=180.0)
    ap.add_argument("--hold", type=float, metavar="SECONDS",
                    help="fixed schedule: give every position exactly this many "
                         "seconds, so an operator who cannot see this output can "
                         "follow it on a clock. Implies --auto.")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=5)
    sys_id, comp = link.target(master)

    print()
    print("Six-position accelerometer calibration.")
    print("  - props off; the aircraft gets turned upside down")
    print("  - USB cable taped to the frame with a service loop: a micro-USB")
    print("    jack works loose exactly when the aircraft is inverted, and a")
    print("    disconnect mid-calibration leaves the old values in place")
    print("  - a flat reference surface; the table's error becomes the")
    print("    aircraft's idea of level")
    if args.hold:
        args.auto = True
    if not args.auto:
        print()
        input("Return to start: ")
    else:
        print()
        print("AUTO mode: hold each position steady and it advances on its own.")
        print("Order: level, left side down, right side down, nose down, nose up, back.")
        print()

    # Register as a GCS first: ArduPilot will not run the interactive
    # calibration for a channel it has never heard a heartbeat from.
    for _ in range(4):
        link.heartbeat(master, interval=0)
        time.sleep(0.3)

    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 0, 0, 0, 1, 0, 0)

    done, failed = False, False
    deadline = time.time() + 900
    answered = set()
    signs = {}

    # ArduPilot stops sending the human-readable "Place vehicle X" text the
    # moment a GCS answers with MAV_CMD_ACCELCAL_VEHICLE_POS:
    #
    #   bool AP_AccelCal::gcs_vehicle_position(float position) {
    #       _use_gcs_snoop = false;        // text prompts off from here on
    #
    # From then on it asks for each position by SENDING COMMAND_LONG with
    # command MAV_CMD_ACCELCAL_VEHICLE_POS and param1 = the step, and reports
    # success or failure the same way. A tool that only watches STATUSTEXT
    # banks position 1 and then waits forever while the board asks, unheard,
    # for position 2 -- exactly what happened on this airframe.
    POS_SUCCESS, POS_FAILED = 16777215, 16777216

    def handle_position(chosen):
        """Guide the operator into `chosen`, then acknowledge it to the board."""
        name = dict(POSITIONS)[chosen]
        if args.hold:
            print("  POSITION %d of 6: %s  -- hold for %.0fs"
                  % (len(answered) + 1, name.upper(), args.hold))
            beep(master, TUNE_START)
            hold_for_position(master, chosen, name, signs, args.hold, args.settle)
            beep(master, TUNE_BANKED)
        elif args.auto:
            print("  POSITION %d of 6: %s" % (len(answered) + 1, name.upper()))
            beep(master, TUNE_START)
            if not wait_for_position(master, chosen, name, signs,
                                     args.settle, args.position_timeout):
                print("  giving up; nothing has been written.")
                return "stop"
            beep(master, TUNE_BANKED)
        else:
            input("  Put the aircraft %s, hold it still, then press Return: "
                  % name.upper())
        master.mav.command_long_send(
            sys_id, comp, link.mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
            0, chosen, 0, 0, 0, 0, 0, 0)
        answered.add(chosen)
        print("    sent '%s' (%d of 6) -- KEEP STILL, the board samples after"
              % (name, len(answered)))
        return "ok"
    while time.time() < deadline and not done and not failed:
        link.heartbeat(master)
        msg = master.recv_match(type=["STATUSTEXT", "COMMAND_ACK", "COMMAND_LONG"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            # Log every calibration ack. The POSITION acknowledgements are the
            # ones that matter: if the board rejects a position it simply stops
            # asking for more, and without this line that is indistinguishable
            # from a board that went quiet for no reason -- which cost two
            # restarts here before anyone thought to look.
            NAMES = {link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                     "PREFLIGHT_CALIBRATION",
                     link.mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
                     "ACCELCAL_VEHICLE_POS"}
            RESULTS = {0: "ACCEPTED", 1: "TEMP REJECTED", 2: "DENIED",
                       3: "UNSUPPORTED", 4: "FAILED", 5: "IN PROGRESS",
                       6: "CANCELLED"}
            if msg.command in NAMES:
                print("    ack: %s -> %s" % (NAMES[msg.command],
                                             RESULTS.get(msg.result, msg.result)))
            if msg.command == link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION \
                    and msg.result not in (0, 5):
                print("the board refused to start the calibration (result %d)." % msg.result)
                print("It will refuse while armed, or while it thinks it is flying.")
                return 1
            continue

        if msg.get_type() == "COMMAND_LONG":
            if msg.command != link.mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
                continue
            requested = int(msg.param1)
            if requested == POS_SUCCESS:
                done = True
                break
            if requested == POS_FAILED:
                failed = True
                break
            if requested in dict(POSITIONS):
                print()
                print("  board asks for position %d (protocol message)" % requested)
                if handle_position(requested) == "stop":
                    return 1
            continue

        text = link.statustext(msg)
        if not text:
            continue
        print("  board: %s" % text)
        low = text.lower()

        # Only calibration messages decide the outcome. ArduPilot emits its
        # usual PreArm nag throughout, and "PreArm: Battery 1 low voltage
        # FAILSAFE" contains the substring "fail" -- which aborted a perfectly
        # good calibration after position 1 the first time this ran. Match on
        # the calibration's own wording, and ignore the unrelated traffic
        # explicitly rather than hoping a substring is specific enough.
        if low.startswith(("prearm", "arm ", "ekf", "ahrs", "initialising",
                           "gps", "compass variance", "u-blox")):
            continue
        if "calibration successful" in low or ("calibration" in low and "success" in low):
            done = True
            break
        if "calibration failed" in low or ("calibration" in low and "fail" in low):
            failed = True
            break

        # The firmware asks for a position, e.g. "Place vehicle level and press
        # any key." Matching the position word is more robust than matching the
        # whole sentence, which has changed wording between releases.
        if "place" in low or "press any key" in low:
            chosen = None
            for word, value in POS_BY_WORD.items():
                if word in low.replace(" ", ""):
                    chosen = value
                    break
            if chosen is None:
                continue
            name = dict(POSITIONS)[chosen]
            if chosen in answered and len(answered) < 6:
                # The board repeats a prompt if it did not like the sample.
                print("    (repeat of '%s' -- hold it steadier)" % name)
            print()
            if handle_position(chosen) == "stop":
                return 1

    print()
    if done:
        beep(master, TUNE_DONE)
        print("calibration successful.")
        for axis in "XYZ":
            offs = link.get_param(master, "INS_ACCOFFS_%s" % axis)
            scal = link.get_param(master, "INS_ACCSCAL_%s" % axis)
            print("  INS_ACCOFFS_%s %s   INS_ACCSCAL_%s %s" % (axis, offs, axis, scal))
        print()
        print("Scale factors should be within a few percent of 1.0 and offsets")
        print("well under 3.5 m/s/s. Then: ./level_calibrate.py, and")
        print("./calibrate_compass.py -- in that order.")
        print()
        print("A reboot is needed before the new values are used in flight.")
        return 0
    if failed:
        print("calibration FAILED. Nothing was saved; the previous values stand.")
        print("Usual causes: the aircraft moved during a position, or a position")
        print("was not actually held in the attitude the board asked for.")
        return 1
    print("timed out waiting for the board.")
    return 1


if __name__ == "__main__":
    sys.exit(link.run(main, "interrupted -- the board may still be mid-calibration. Power-cycle it."))
