#!/usr/bin/env python3
"""Everything at once, split into what stops a flight and what merely worries.

    ./preflight.py                 # the full set
    ./preflight.py --quick         # skip the stick sweep and the long samples

Design rule, learned by getting it wrong on the APM 2.8: never report a fault
where the test did not actually look. --quick skips the stick sweep, so it
cannot know whether the mode switch reaches three positions -- and it says
UNKNOWN rather than inventing a blocker. A measurement that cannot support a
conclusion must not produce one in either direction.

The board's own PreArm messages are treated as authoritative. They come from
the code that will actually refuse to arm, so anything it says is a blocker
here too, whatever this tool's own opinion is.

airframe.local.json, if present, records deliberate choices that no parameter
can express -- e.g. which compass is physically on the mast. Without it, a
correctly configured aircraft trips the same warning forever.
"""

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "lib"))

import cubelink as link  # noqa: E402

LOCAL_FILE = os.path.join(HERE, "airframe.local.json")


class Checks:
    def __init__(self):
        self.blockers, self.warnings, self.unknowns, self.oks = [], [], [], []

    def blocker(self, what, why):
        self.blockers.append((what, why))
        print("  BLOCK  %-26s %s" % (what, why))

    def warn(self, what, why):
        self.warnings.append((what, why))
        print("  warn   %-26s %s" % (what, why))

    def unknown(self, what, why):
        self.unknowns.append((what, why))
        print("  ?      %-26s %s" % (what, why))

    def ok(self, what, why=""):
        self.oks.append((what, why))
        print("  ok     %-26s %s" % (what, why))


def local_notes():
    if not os.path.exists(LOCAL_FILE):
        return {}
    try:
        with open(LOCAL_FILE) as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        print("  (could not read airframe.local.json: %s)" % exc)
        return {}


def section(title):
    print()
    print("-- %s %s" % (title, "-" * max(0, 64 - len(title))))


def main():
    ap = argparse.ArgumentParser(description="pre-flight check")
    ap.add_argument("--port")
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2

    checks = Checks()
    notes = local_notes()
    banner = link.parse_banner(link.request_banner(master))
    link.request_streams(master, rate_hz=10)

    section("link")
    if not link.telemetry_alive(master, seconds=4.0):
        checks.blocker("telemetry", "no streams at all -- the main loop is not running")
        print()
        print("Nothing below can be measured. Power-cycle the board and retry.")
        return 1
    checks.ok("firmware", "%s %s (%s)" % (banner.get("vehicle", "?"),
                                          banner.get("version", "?"),
                                          banner.get("fw_hash", "?")))

    window = 4.0 if args.quick else 10.0
    latest, counts, texts = link.collect(master, seconds=window)
    params = link.fetch_params(master)
    params.pop("_count", None)

    def p(*names):
        for name in names:
            if name in params:
                return params[name]
        return None

    # ---------------------------------------------------- the board's opinion
    section("what the board itself refuses")
    prearm = [t for t in texts if t.startswith(("PreArm", "Arm"))]
    if not prearm:
        prearm = link.prearm_messages(master, seconds=8.0)
    if prearm:
        for text in dict.fromkeys(prearm):
            checks.blocker("pre-arm", text)
    else:
        checks.ok("pre-arm", "no refusal seen in this window")
        print("         (ArduPilot repeats these every ~30s while disarmed; a")
        print("          clean window is good evidence, not proof)")

    # ------------------------------------------------------------- estimator
    section("estimator")
    ekf = latest.get("EKF_STATUS_REPORT")
    if ekf is None:
        checks.unknown("EKF", "no EKF_STATUS_REPORT in this window")
    else:
        worst = max(ekf.velocity_variance, ekf.pos_horiz_variance,
                    ekf.pos_vert_variance, ekf.compass_variance)
        if worst > 1.0:
            checks.blocker("EKF variance", "%.2f -- the estimator does not trust itself" % worst)
        elif worst > 0.5:
            checks.warn("EKF variance", "%.2f, high" % worst)
        else:
            checks.ok("EKF variance", "%.2f" % worst)

    # ------------------------------------------------------------ attitude
    section("attitude and IMU")
    rolls, pitches = [], []
    deadline = time.time() + (2.0 if args.quick else 5.0)
    while time.time() < deadline:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
        if msg is not None:
            rolls.append(math.degrees(msg.roll))
            pitches.append(math.degrees(msg.pitch))
    if not rolls:
        checks.unknown("attitude", "no ATTITUDE messages")
    else:
        roll = sum(rolls) / len(rolls)
        pitch = sum(pitches) / len(pitches)
        sd = max(
            math.sqrt(sum((v - roll) ** 2 for v in rolls) / len(rolls)),
            math.sqrt(sum((v - pitch) ** 2 for v in pitches) / len(pitches)))
        if max(abs(roll), abs(pitch)) > 2.0:
            checks.blocker("level", "%+.1f / %+.1f deg at rest -- it will drift sideways"
                           % (roll, pitch))
        elif max(abs(roll), abs(pitch)) > 1.0:
            checks.warn("level", "%+.1f / %+.1f deg -- ./level_calibrate.py" % (roll, pitch))
        else:
            checks.ok("level", "%+.2f / %+.2f deg" % (roll, pitch))
        if sd > 0.5:
            checks.warn("attitude steadiness", "sd %.2f deg -- something is rocking it" % sd)

    if not any(p("INS_ACCOFFS_%s" % ax) for ax in "XYZ"):
        checks.blocker("accel calibration", "INS_ACCOFFS_* are zero -- never calibrated")
    else:
        checks.ok("accel calibration", "offsets present")

    vib = latest.get("VIBRATION")
    if vib is not None:
        worst = max(vib.vibration_x, vib.vibration_y, vib.vibration_z)
        clips = vib.clipping_0 + vib.clipping_1 + vib.clipping_2
        if clips:
            checks.blocker("IMU clipping", "%d clip events -- the IMU is saturating" % clips)
        elif worst > 30:
            checks.blocker("vibration", "%.0f m/s/s" % worst)
        elif worst > 15:
            checks.warn("vibration", "%.0f m/s/s -- check props and mount" % worst)
        else:
            checks.ok("vibration", "%.1f m/s/s" % worst)

    # ------------------------------------------------------------- compass
    section("compass")
    declared = notes.get("compass")
    any_mag = False
    for n, suffix in ((1, ""), (2, "2"), (3, "3")):
        devid = p("COMPASS_DEV_ID%s" % suffix, "COMPASS_PRIO%d_ID" % n)
        info = link.decode_devid(devid or 0, link.COMPASS_DEVTYPES)
        if info is None:
            continue
        any_mag = True
        ofs = [p("COMPASS_OFS%s_%s" % (suffix, ax)) or 0 for ax in "XYZ"]
        mag = sum(v * v for v in ofs) ** 0.5
        use = p("COMPASS_USE" if n == 1 else "COMPASS_USE%d" % n)
        if not use:
            checks.ok("compass %d" % n, "present but COMPASS_USE%s=0, ignored" % suffix)
            continue
        if mag == 0:
            checks.blocker("compass %d calibration" % n, "offsets are zero -- never calibrated")
        elif mag > 600:
            checks.blocker("compass %d offsets" % n,
                           "|%.0f| exceeds the 600 pre-arm limit" % mag)
        elif mag > 400:
            checks.warn("compass %d offsets" % n, "|%.0f|, high but passable" % mag)
        else:
            checks.ok("compass %d" % n, "%s, offsets |%.0f|" % (info["name"], mag))
    if not any_mag:
        checks.blocker("compass", "none detected -- no heading source but the GPS")

    # A compass that is configured for use but has no device behind it. Normal
    # on a board with fewer compasses than COMPASS_USE* slots -- but it is
    # also what an INTERMITTENT sensor looks like, and this airframe has one:
    # the onboard LSM303D was probed on the first boot after the 4.7.1 flash
    # and not on later ones. A soft reboot does not power-cycle an SPI sensor,
    # so compare against a real power-cycle before calling it dead.
    empty = [n for n, suffix in ((2, "2"), (3, "3"))
             if p("COMPASS_USE%s" % suffix) and p("COMPASS_DEV_ID%s" % suffix) == 0]
    if empty:
        checks.ok("empty compass slots",
                  "USE set with no device on %s -- firmware default, harmless"
                  % ", ".join("slot %d" % n for n in empty))
        print("         (but a slot that HAS held a device and is now empty means an")
        print("          intermittent sensor. A soft reboot does not power-cycle an")
        print("          SPI sensor -- pull USB and compare. The onboard LSM303D on")
        print("          this airframe probed once after the 4.7.1 flash and not since.)")
    if declared:
        checks.ok("compass choice", "airframe.local.json says: %s" % declared)

    # ----------------------------------------------------------------- GPS
    section("GPS")
    gps = latest.get("GPS_RAW_INT")
    needs_gps = any(int(p("FLTMODE%d" % n) or 0) in (3, 5, 6, 16, 21)
                    for n in range(1, 7))
    if gps is None or gps.fix_type == 0:
        if needs_gps:
            checks.blocker("GPS", "not detected, but a configured flight mode needs it")
        else:
            checks.warn("GPS", "not detected (no configured mode needs it)")
    elif gps.fix_type < 3:
        (checks.blocker if needs_gps else checks.warn)(
            "GPS fix", "%s, %d sats" % (link.GPS_FIX_TYPES.get(gps.fix_type),
                                        gps.satellites_visible))
    else:
        hdop = gps.eph / 100.0
        if gps.satellites_visible < 8 or hdop > 2.0:
            checks.warn("GPS quality", "%d sats, HDOP %.2f -- position modes unreliable"
                        % (gps.satellites_visible, hdop))
        else:
            checks.ok("GPS", "3D fix, %d sats, HDOP %.2f" % (gps.satellites_visible, hdop))

    # ------------------------------------------------------------- battery
    section("battery")
    monitor = int(p("BATT_MONITOR") or 0)
    sys_status = latest.get("SYS_STATUS")
    volts = sys_status.voltage_battery / 1000.0 if sys_status else 0.0
    low = p("BATT_LOW_VOLT")
    if not monitor:
        checks.blocker("BATT_MONITOR", "0 -- no voltage sensing, so no battery failsafe")
    else:
        state, message = link.battery_verdict(volts, low)
        if state == "absent":
            checks.blocker("pack voltage", message)
        elif state == "low":
            checks.blocker("pack voltage", message)
        else:
            checks.ok("pack voltage", message)

    # ------------------------------------------------------------------ RC
    section("RC and modes")
    rc = latest.get("RC_CHANNELS")
    rc_values = []
    if rc is not None:
        rc_values = [getattr(rc, "chan%d_raw" % n) for n in range(1, 9)]
    if not rc_values or all(v == 0 for v in rc_values):
        checks.blocker("RC input", "no channel decoding")
    else:
        checks.ok("RC input", " ".join(str(v) for v in rc_values[:6]))
        rc3_min = p("RC3_MIN")
        fs_value = p("FS_THR_VALUE")
        if p("RC1_MIN") == 1100 and p("RC1_MAX") == 1900:
            checks.warn("radio calibration", "endpoints are firmware defaults")
        if rc3_min and fs_value:
            if fs_value >= rc3_min:
                checks.blocker("FS_THR_VALUE",
                               "%g is not below RC3_MIN %g -- failsafe will misfire"
                               % (fs_value, rc3_min))
            else:
                checks.ok("throttle failsafe", "FS_THR_VALUE %g < RC3_MIN %g"
                          % (fs_value, rc3_min))

    modes = [int(p("FLTMODE%d" % n) or 0) for n in range(1, 7)]
    recovery = [m for m in modes if m in (6, 9, 21)]
    if not recovery:
        checks.warn("flight modes", "no RTL, LAND or SMART_RTL on any slot -- there is "
                                    "no way to hand control back")
    else:
        checks.ok("flight modes", "slots %s" % " ".join(str(m) for m in modes))

    if args.quick:
        checks.unknown("mode switch reach",
                       "--quick skipped the sweep, so this run cannot tell")
    else:
        ch = int(p("FLTMODE_CH") or 5)
        print()
        print("  Sweep the mode switch through every position now (8s)...")
        seen = set()
        deadline = time.time() + 8
        while time.time() < deadline:
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
            if msg is None:
                continue
            value = getattr(msg, "chan%d_raw" % ch, 0)
            if value:
                seen.add(value // 100)
        if len(seen) >= 3:
            checks.ok("mode switch reach", "%d distinct positions seen" % len(seen))
        elif len(seen) == 2:
            checks.warn("mode switch reach",
                        "only 2 positions -- a 2-position switch on ch%d" % ch)
        else:
            checks.unknown("mode switch reach",
                           "the switch was not moved, so this says nothing")

    # --------------------------------------------------------------- logging
    section("logging and safety")
    if sys_status is not None:
        if not sys_status.onboard_control_sensors_health & 0x1000000:
            checks.warn("logging", "unhealthy -- no SD card, or it cannot be mounted")
        else:
            checks.ok("logging", "healthy")
        if not sys_status.onboard_control_sensors_enabled & 0x8000:
            checks.ok("safety switch", "engaged -- outputs inhibited (bench-safe)")
        else:
            checks.warn("safety switch", "disengaged -- outputs are LIVE")

    # --------------------------------------------------------------- verdict
    print()
    print("=" * 70)
    if checks.blockers:
        print("DO NOT FLY -- %d blocker(s):" % len(checks.blockers))
        for what, why in checks.blockers:
            print("  * %-24s %s" % (what, why))
    else:
        print("No blockers found.")
    if checks.warnings:
        print()
        print("%d warning(s):" % len(checks.warnings))
        for what, why in checks.warnings:
            print("  - %-24s %s" % (what, why))
    if checks.unknowns:
        print()
        print("%d thing(s) this run could not determine:" % len(checks.unknowns))
        for what, why in checks.unknowns:
            print("  ? %-24s %s" % (what, why))
    print("=" * 70)
    return 1 if checks.blockers else 0


if __name__ == "__main__":
    sys.exit(link.run(main))
