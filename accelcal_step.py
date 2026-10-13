#!/usr/bin/env python3
"""Drive the six-position accel calibration ONE POSITION PER INVOCATION.

    ./accelcal_step.py start     # begin, and report the first position wanted
    ./accelcal_step.py confirm   # verify the aircraft is in it, bank it, report the next
    ./accelcal_step.py status    # what is it waiting for right now
    ./accelcal_step.py abort

Why this exists alongside calibrate_accel.py: the calibration state lives on
the BOARD, not in this process, so it survives between invocations. That
makes a conversational workflow possible -- the operator is told one
position, does it, and the next command verifies and banks it -- which is the
only shape that works when whoever is holding the aircraft cannot see the
tool's output.

Each confirm refuses to bank a position unless the accelerometer agrees the
aircraft is actually in it: gravity on the expected axis, the reading still,
and the sign opposite to the paired position. A six-position calibration that
completes on samples taken in the wrong attitude is worse than no
calibration, because the pre-arm check then passes.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "lib"))
sys.path.insert(0, HERE)

import cubelink as link          # noqa: E402
import calibrate_accel as ac     # noqa: E402  (shares the axis/steadiness rules)

STATE = os.path.join(HERE, "logs", "accelcal-state.json")

HOW = {
    1: "LEVEL -- sitting normally on a flat surface",
    2: "LEFT SIDE DOWN -- roll it onto its left side",
    3: "RIGHT SIDE DOWN -- roll it onto its right side",
    4: "NOSE DOWN -- point the nose at the floor",
    5: "NOSE UP -- point the nose at the ceiling",
    6: "ON ITS BACK -- upside down",
}


def load():
    try:
        with open(STATE) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {"position": None, "signs": {}, "done": []}


def save(state):
    os.makedirs(os.path.dirname(STATE), exist_ok=True)
    with open(STATE, "w") as fh:
        json.dump(state, fh, indent=2)


def announce(master, position):
    print()
    print("=" * 58)
    print("  NEXT POSITION (%d of 6): %s" % (position, HOW[position]))
    print("=" * 58)
    link_beep(master, ac.TUNE_START)


def link_beep(master, tune):
    ac.beep(master, tune)


def wait_for_prompt(master, seconds=20):
    """Listen for the board's next 'place vehicle X' prompt. Returns 1-6, or
    'done' / 'failed' / None."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        link.heartbeat(master)
        msg = master.recv_match(type=["STATUSTEXT", "COMMAND_ACK"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            continue
        text = link.statustext(msg)
        if not text:
            continue
        low = text.lower()
        if low.startswith(("prearm", "arm ", "ekf", "ahrs", "initialising",
                           "gps", "u-blox", "compass variance", "battery")):
            continue
        print("  board: %s" % text)
        if "calibration successful" in low or ("calibration" in low and "success" in low):
            return "done"
        if "calibration failed" in low or ("calibration" in low and "fail" in low):
            return "failed"
        if "place" in low or "press any key" in low:
            squashed = low.replace(" ", "")
            for word, value in ac.POS_BY_WORD.items():
                if word in squashed:
                    return value
    return None


def do_start(master):
    for _ in range(5):
        link.heartbeat(master, interval=0)
        time.sleep(0.3)
    sys_id, comp = link.target(master)
    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 0, 0, 0, 1, 0, 0)
    print("calibration start sent; waiting for the board to ask for a position")
    result = wait_for_prompt(master, 25)
    if result in (None, "done", "failed"):
        print("the board did not ask for a position (%s)." % result)
        print("If it stays silent, power-cycle the board and run 'start' again.")
        return 1
    save({"position": result, "signs": {}, "done": []})
    announce(master, result)
    print()
    print("Put the aircraft there, hold it still, then tell me and I will check it.")
    return 0


def do_confirm(master):
    state = load()
    position = state.get("position")
    if not position:
        print("no calibration in progress -- run 'start' first.")
        return 1

    print("checking the aircraft is in position %d (%s)" % (position, HOW[position]))
    axis = ac.POSITION_AXIS[position]
    index = {"x": 0, "y": 1, "z": 2}[axis]
    signs = {int(k): v for k, v in state.get("signs", {}).items()}
    opposes = ac.POSITION_OPPOSES.get(position)
    want_opposite_of = signs.get(opposes) if opposes else None

    window = []
    deadline = time.time() + 12
    verdict = None
    while time.time() < deadline:
        link.heartbeat(master)
        reading = ac.read_accel(master)
        if reading is None:
            continue
        window.append((time.time(), reading))
        window[:] = [w for w in window if w[0] > time.time() - 1.5]
        if len(window) < 5:
            continue
        spans = [max(w[1][i] for w in window) - min(w[1][i] for w in window)
                 for i in range(3)]
        value = reading[index]
        others = [abs(v) for i, v in enumerate(reading) if i != index]
        on_axis = abs(value) > ac.DOMINANT_MIN and abs(value) > max(others) * 1.5
        sign = 1 if value > 0 else -1
        sign_ok = want_opposite_of is None or sign == -want_opposite_of
        steady = max(spans) <= ac.STEADY_BAND
        print("  x%+5d y%+5d z%+5d   on-%s=%s steady=%s(%dmG) distinct=%s"
              % (reading[0], reading[1], reading[2], axis.upper(),
                 on_axis, steady, max(spans), sign_ok))
        if on_axis and steady and sign_ok:
            verdict = (value, sign)
            break
        time.sleep(0.5)

    if verdict is None:
        reading = ac.read_accel(master) or (0, 0, 0)
        value = reading[index]
        print()
        print("NOT ACCEPTED -- nothing banked, the board is still waiting.")
        if abs(value) <= ac.DOMINANT_MIN:
            print("  Gravity is not on the %s axis: the aircraft does not look like"
                  % axis.upper())
            print("  it is %s." % HOW[position])
        elif want_opposite_of is not None and (1 if value > 0 else -1) != -want_opposite_of:
            print("  That is the SAME side as the previous position -- turn it the")
            print("  other way.")
        else:
            print("  It is still moving. Rest it on something and try again.")
        link_beep(master, ac.TUNE_MOVING)
        return 1

    value, sign = verdict
    sys_id, comp = link.target(master)
    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
        0, position, 0, 0, 0, 0, 0, 0)
    link_beep(master, ac.TUNE_BANKED)
    print()
    print("ACCEPTED: %s axis %+d, steady. Position %d banked."
          % (axis.upper(), value, position))

    signs[position] = sign
    done = state.get("done", []) + [position]
    result = wait_for_prompt(master, 20)
    if result == "done":
        save({"position": None, "signs": {}, "done": []})
        link_beep(master, ac.TUNE_DONE)
        print()
        print("ALL SIX POSITIONS DONE -- calibration successful.")
        for ax in "XYZ":
            print("  INS_ACCOFFS_%s %s   INS_ACCSCAL_%s %s"
                  % (ax, link.get_param(master, "INS_ACCOFFS_%s" % ax),
                     ax, link.get_param(master, "INS_ACCSCAL_%s" % ax)))
        return 0
    if result == "failed":
        save({"position": None, "signs": {}, "done": []})
        print()
        print("the board reported the calibration FAILED. Nothing was saved.")
        return 1
    if result is None:
        save({"position": position, "signs": signs, "done": done})
        print("banked, but the board has not asked for the next position yet.")
        print("Run 'status' in a moment.")
        return 0
    save({"position": result, "signs": signs, "done": done})
    print("  %d of 6 banked." % len(done))
    announce(master, result)
    return 0


def do_status(master):
    state = load()
    if not state.get("position"):
        print("no calibration in progress.")
        return 0
    print("%d of 6 banked. Waiting for: %s"
          % (len(state.get("done", [])), HOW[state["position"]]))
    reading = ac.read_accel(master)
    if reading:
        print("  accel now: x%+5d y%+5d z%+5d" % reading)
    return 0


def main():
    ap = argparse.ArgumentParser(description="stepwise accel calibration")
    ap.add_argument("action", choices=["start", "confirm", "status", "abort"])
    ap.add_argument("--port")
    args = ap.parse_args()

    master = link.connect(args.port)
    link.request_streams(master, rate_hz=20)

    if args.action == "abort":
        save({"position": None, "signs": {}, "done": []})
        print("state cleared. Power-cycle the board to stop the board-side routine.")
        return 0
    if args.action == "start":
        return do_start(master)
    if args.action == "confirm":
        return do_confirm(master)
    return do_status(master)


if __name__ == "__main__":
    sys.exit(link.run(main))
