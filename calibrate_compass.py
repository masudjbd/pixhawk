#!/usr/bin/env python3
"""Run ArduPilot's onboard magnetometer calibration, and judge the result.

    ./calibrate_compass.py              # calibrate every compass, then accept
    ./calibrate_compass.py --no-save    # measure and report, write nothing

This is the 4.x routine (MAG_CAL), not the sphere fit the APM 2.8 toolkit had
to implement itself. The firmware does the fitting, including soft-iron
(COMPASS_DIA/ODI) and -- with COMPASS_AUTO_ROT -- the external compass's
orientation, which on the old board needed a separate tool and a gyro-mag
cross-product solve.

What this tool adds over pressing the button in a GCS: it refuses to let a
bad fit pass quietly. It prints the fitness number, the offsets, and what
each means, and it will not accept a calibration whose offsets exceed the
pre-arm limit.

How to move the aircraft:

  Rotate it through every attitude: hold each face of the aircraft toward the
  ground in turn and rotate 360 degrees about that axis. The firmware needs
  samples spread over a sphere; it will sit at the same percentage forever if
  you only yaw it.

  TURN ONE WAY, THEN BACK THE OTHER WAY. Continuous same-direction spinning
  tangles the USB cable. Magnetically it makes no difference -- the sensor
  only cares which directions it has pointed.

  THE AIRCRAFT MUST BE ASSEMBLED, with the battery in its normal place if you
  can manage it. Hard iron comes from the ESCs, the wiring and the battery,
  so calibrating a bare board measures a magnetic environment the aircraft
  will never fly in.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

CAL_STATUS = {
    0: "not started", 1: "waiting to start", 2: "running step one",
    3: "running step two", 4: "success", 5: "FAILED", 6: "bad orientation",
    7: "bad radius",
}


def main():
    ap = argparse.ArgumentParser(description="onboard compass calibration")
    ap.add_argument("--port")
    ap.add_argument("--no-save", action="store_true",
                    help="run and report, but do not accept the result")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--yes", "-y", action="store_true",
                    help="start immediately, no prompt. For driving this from a "
                         "session with no keyboard at the aircraft.")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=5)
    sys_id, comp = link.target(master)

    auto_rot = link.get_param(master, "COMPASS_AUTO_ROT")
    print()
    print("COMPASS_AUTO_ROT %s%s" % (
        auto_rot, " -- orientation will be fitted automatically" if auto_rot == 2 else ""))
    print()
    print("Rotate the aircraft through every attitude. One way, then back the")
    print("other way, so the USB lead does not wind up. Props off.")
    print()
    if not args.yes:
        input("Return to start: ")
    else:
        print("--yes: starting now. Begin rotating the aircraft.")

    # param1 0 = every compass, param2 retry, param3 autosave (0: we decide),
    # param4 delay, param5 autoreboot.
    for _ in range(4):
        link.heartbeat(master, interval=0)
        time.sleep(0.3)

    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
        0, 0, 1, 0, 0, 0, 0, 0)

    reports = {}
    last_pct = {}
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        link.heartbeat(master)
        msg = master.recv_match(
            type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT", "COMMAND_ACK"],
            blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "COMMAND_ACK":
            if msg.command == link.mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL \
                    and msg.result != 0:
                print("the board refused to start (result %d)." % msg.result)
                return 1
        elif kind == "STATUSTEXT":
            text = link.statustext(msg)
            if text:
                print("  board: %s" % text)
        elif kind == "MAG_CAL_PROGRESS":
            pct = msg.completion_pct
            if last_pct.get(msg.compass_id) != pct and pct % 5 == 0:
                last_pct[msg.compass_id] = pct
                print("  compass %d  %3d%%" % (msg.compass_id, pct))
        elif kind == "MAG_CAL_REPORT":
            reports[msg.compass_id] = msg
            print("  compass %d finished: %s, fitness %.1f" % (
                msg.compass_id, CAL_STATUS.get(msg.cal_status, msg.cal_status),
                msg.fitness))
            # One report per compass; stop once every running compass is in.
            if len(reports) >= max(1, len(last_pct)):
                break

    if not reports:
        print()
        print("no MAG_CAL_REPORT arrived. Nothing was changed.")
        return 1

    print()
    good = True
    for cid in sorted(reports):
        r = reports[cid]
        mag = (r.ofs_x ** 2 + r.ofs_y ** 2 + r.ofs_z ** 2) ** 0.5
        print("compass %d" % cid)
        print("  status     %s" % CAL_STATUS.get(r.cal_status, r.cal_status))
        print("  fitness    %.1f  (lower is better; under 8 is good, over 16 poor)" % r.fitness)
        print("  offsets    x %+.0f  y %+.0f  z %+.0f   magnitude %.0f" % (
            r.ofs_x, r.ofs_y, r.ofs_z, mag))
        print("  scale      %.3f" % r.scale_factor)
        print("  autosaved  %s" % bool(r.autosaved))
        if r.cal_status != 4:
            good = False
            print("  -> did not succeed. Nothing to accept for this compass.")
        if mag > 600:
            good = False
            print("  -> offsets over 600 FAIL the pre-arm check. This almost always")
            print("     means the compass sits too close to power wiring: move it")
            print("     further up the mast, or away from the ESC leads, and redo.")
        elif mag > 400:
            print("  -> offsets are high but passable. Worth moving the compass")
            print("     away from the battery and ESC leads if you can.")
        if r.fitness > 16:
            print("  -> poor fit. Usually incomplete coverage: the aircraft was")
            print("     not rotated through enough attitudes.")

    if args.no_save:
        print()
        print("--no-save: not accepting. The board keeps its previous offsets.")
        master.mav.command_long_send(
            sys_id, comp, link.mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
            0, 0, 0, 0, 0, 0, 0, 0)
        return 0 if good else 1

    if not good:
        print()
        print("not accepting a calibration with the problems above. Fix the cause")
        print("and run it again -- accepting a bad fit is worse than no fit,")
        print("because the pre-arm check will then pass.")
        master.mav.command_long_send(
            sys_id, comp, link.mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
            0, 0, 0, 0, 0, 0, 0, 0)
        return 1

    master.mav.command_long_send(
        sys_id, comp, link.mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
        0, 0, 0, 0, 0, 0, 0, 0)
    print()
    print("accepted. Reading back what was stored:")
    for n, suffix in ((1, ""), (2, "2"), (3, "3")):
        ofs = [link.get_param(master, "COMPASS_OFS%s_%s" % (suffix, ax)) for ax in "XYZ"]
        if all(v is None for v in ofs):
            continue
        orient = link.get_param(master, "COMPASS_ORIENT%s" % suffix)
        print("  compass %d  offsets %s  ORIENT %s" % (
            n, " ".join("%+.0f" % (v or 0) for v in ofs), orient))
    print()
    print("Then: rotate the aircraft and check the heading in a GCS points the")
    print("right way. A calibration can be numerically perfect and still have")
    print("the wrong orientation, and nothing on the bench catches that except")
    print("looking at the heading.")
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
