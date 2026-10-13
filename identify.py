#!/usr/bin/env python3
"""Read-only inventory of a Pixhawk 2.1 Cube: what is on it, and what it needs.

Writes nothing. Everything after this depends on its output -- which board it
actually is, which sensors enumerated, which bus each one is on, what the
frame and motor mapping say, and which of the bring-up steps are still
outstanding.

    ./identify.py                # full: parameter table + live telemetry
    ./identify.py --quick        # live telemetry only, no parameter dump
    ./identify.py --port /dev/cu.usbmodem01

The parameter table is fetched whole (~1100 values, 20-40s on USB) because
almost every question below is answered by a group of parameters rather than
one, and because a dropped PARAM_VALUE is indistinguishable from a parameter
the firmware does not have. fetch_params re-requests the gaps; a single pass
would not.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402


def hr(title):
    print()
    print("== %s %s" % (title, "=" * max(0, 68 - len(title))))


def fmt(value, spec="%g"):
    return "--" if value is None else spec % value


def enum(value, table, unknown="unknown"):
    if value is None:
        return "--"
    return table.get(int(value), "%s (%g)" % (unknown, value))


def main():
    ap = argparse.ArgumentParser(description="read-only Cube inventory")
    ap.add_argument("--port")
    ap.add_argument("--quick", action="store_true",
                    help="skip the parameter dump (live telemetry only)")
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="telemetry listen window (default 6)")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2

    todo = []

    # ---------------------------------------------------------------- identity
    hr("identity")
    banner = link.parse_banner(link.request_banner(master))
    board_id = banner.get("board_id")
    print("  board id       %s" % (board_id or "not reported"))
    if board_id:
        print("  board          %s" % link.BOARD_IDS.get(board_id, "unrecognised board id"))
    print("  firmware       %s %s (%s)" % (banner.get("vehicle", "?"),
                                           banner.get("version", "?"),
                                           banner.get("fw_hash", "?")))
    if banner.get("os"):
        print("  os             %s" % banner["os"])
    if banner.get("cpu_uid"):
        print("  cpu uid        %s" % banner["cpu_uid"])
    if banner.get("frame_banner"):
        print("  frame (banner) %s" % banner["frame_banner"])

    version = link.version_tuple(banner.get("version"))
    ver = link.autopilot_version(master)
    if ver is not None and not version:
        fv = ver.flight_sw_version
        version = ((fv >> 24) & 255, (fv >> 16) & 255, (fv >> 8) & 255)
        print("  firmware       %d.%d.%d (AUTOPILOT_VERSION)" % version)
    if ver is not None:
        print("  capabilities   0x%x" % ver.capabilities)
    for extra in banner.get("other", []):
        print("  banner         %s" % extra)

    if version and version < (4, 0, 0):
        todo.append("Firmware is %d.%d.%d. ArduCopter 4.x is available for this board "
                    "(fmuv3/CubeBlack) and brings EKF3, the harmonic notch filter, "
                    "DShot and per-IMU device IDs. Decide before calibrating: a "
                    "major-version change rewrites parameter defaults."
                    % version)

    # ------------------------------------------------------------- parameters
    params = {}
    if not args.quick:
        hr("parameter table")
        params = link.fetch_params(master)
        if params.get("_count"):
            print("  firmware exposes %d parameters" % params["_count"])

    def p(name):
        return params.get(name)

    # ------------------------------------------------------------------ frame
    if params:
        hr("frame and motor outputs")
        fclass, ftype = p("FRAME_CLASS"), p("FRAME_TYPE")
        print("  FRAME_CLASS    %s" % enum(fclass, link.FRAME_CLASSES))
        print("  FRAME_TYPE     %s" % enum(ftype, link.FRAME_TYPES))
        if not fclass:
            todo.append("FRAME_CLASS is 0: the board has no motor mixer, so no output "
                        "is a motor and it cannot arm. Set it before any motor test.")

        print("  MOT_PWM_TYPE   %s" % enum(p("MOT_PWM_TYPE"), link.MOT_PWM_TYPES))
        print("  MOT_PWM_MIN/MAX %s / %s" % (fmt(p("MOT_PWM_MIN")), fmt(p("MOT_PWM_MAX"))))
        print("  MOT_SPIN_ARM   %s      MOT_SPIN_MIN %s" %
              (fmt(p("MOT_SPIN_ARM")), fmt(p("MOT_SPIN_MIN"))))
        print("  MOT_THST_EXPO  %s      MOT_BAT_VOLT_MIN/MAX %s / %s" %
              (fmt(p("MOT_THST_EXPO")), fmt(p("MOT_BAT_VOLT_MIN")),
               fmt(p("MOT_BAT_VOLT_MAX"))))
        if p("MOT_PWM_MIN") in (0, None) or p("MOT_PWM_MAX") in (0, None):
            print("  note           MOT_PWM_MIN/MAX 0 means the mixer uses RC3_MIN/MAX")

        print()
        print("  output  function              trim/min/max")
        for n in range(1, 15):
            func = p("SERVO%d_FUNCTION" % n)
            if func is None:
                continue
            rail = "MAIN %d" % n if n <= 8 else "AUX %d" % (n - 8)
            print("  %-7s %-22s %s / %s / %s" % (
                rail, enum(func, link.SERVO_FUNCTIONS, "function"),
                fmt(p("SERVO%d_TRIM" % n)), fmt(p("SERVO%d_MIN" % n)),
                fmt(p("SERVO%d_MAX" % n))))

        # ------------------------------------------------------------- IMUs
        hr("IMUs")
        acc_ids = link.grep_params(params, r"^INS_ACC\d*_ID$")
        gyr_ids = link.grep_params(params, r"^INS_GYR\d*_ID$")
        if acc_ids or gyr_ids:
            for name in acc_ids + gyr_ids:
                table = link.IMU_DEVTYPES
                print("  %-14s %s" % (name, link.devid_str(link.decode_devid(p(name), table))))
        else:
            print("  this firmware does not expose per-IMU device IDs "
                  "(added in 4.0); counting from live telemetry instead")
        for idx, suffix in ((1, ""), (2, "2"), (3, "3")):
            use = p("INS_USE%s" % suffix)
            offs = [p("INS_ACC%sOFFS_%s" % (suffix, ax)) for ax in "XYZ"]
            scal = [p("INS_ACC%sSCAL_%s" % (suffix, ax)) for ax in "XYZ"]
            if use is None and all(v is None for v in offs):
                continue
            calibrated = any(v not in (None, 0.0) for v in offs) or \
                any(v not in (None, 0.0, 1.0) for v in scal)
            print("  IMU%d           INS_USE%s=%s  accel offsets %s  %s" % (
                idx, suffix, fmt(use), "set" if calibrated else "ZERO",
                "" if calibrated else "<- not calibrated"))
        if not any(p("INS_ACCOFFS_%s" % ax) for ax in "XYZ"):
            todo.append("Accelerometers are not calibrated (INS_ACCOFFS_* are 0). "
                        "Run ./calibrate_accel.py -- six positions on a flat surface.")

        # ----------------------------------------------------------- compass
        hr("compasses")
        found_any = False
        for idx, suffix in ((1, ""), (2, "2"), (3, "3")):
            devid = p("COMPASS_DEV_ID%s" % suffix)
            if devid is None:
                continue
            info = link.decode_devid(devid, link.COMPASS_DEVTYPES)
            if info is None:
                print("  compass %d      none detected" % idx)
                continue
            found_any = True
            ofs = [p("COMPASS_OFS%s_%s" % (suffix, ax)) for ax in "XYZ"]
            mag = sum(v * v for v in ofs if v is not None) ** 0.5
            print("  compass %d      %s" % (idx, link.devid_str(info)))
            print("                 USE=%s  EXTERNAL=%s  ORIENT=%s  offsets |%.0f|" % (
                fmt(p("COMPASS_USE%s" % suffix)),
                fmt(p("COMPASS_EXTERNAL" if suffix == "" else "COMPASS_EXTERN%s" % suffix)),
                fmt(p("COMPASS_ORIENT%s" % suffix)), mag))
            if mag == 0:
                todo.append("Compass %d has zero offsets -- never calibrated. "
                            "Run ./calibrate_compass.py." % idx)
            elif mag > 600:
                todo.append("Compass %d offsets are |%.0f|, over the 600 pre-arm limit. "
                            "It is mounted too close to power wiring." % (idx, mag))
            if info["bus_type"] == "I2C" and \
                    not p("COMPASS_EXTERNAL" if suffix == "" else "COMPASS_EXTERN%s" % suffix):
                todo.append("Compass %d was probed on an I2C bus but COMPASS_EXTERNAL%s is 0. "
                            "If this is the GPS-mast magnetometer the flag is wrong, and "
                            "orientation handling will be wrong with it."
                            % (idx, suffix))
        if not found_any:
            todo.append("No magnetometer detected at all. Without one, heading comes from "
                        "the GPS only: LOITER, RTL and AUTO are unusable.")

        # ------------------------------------------------------------- GPS
        hr("GPS and serial ports")
        # GPS_TYPE/GPS_TYPE2 were renamed GPS1_TYPE/GPS2_TYPE in 4.6. Ask for
        # both rather than reporting "--" on a board that has the parameter
        # under the other spelling.
        for label, names in (("GPS 1", ("GPS1_TYPE", "GPS_TYPE")),
                             ("GPS 2", ("GPS2_TYPE", "GPS_TYPE2"))):
            for name in names:
                if p(name) is not None:
                    print("  %-14s %s (%s)" % (label, enum(p(name), link.GPS_TYPES), name))
                    break
            else:
                print("  %-14s --" % label)
        for n in range(0, 6):
            proto, baud = p("SERIAL%d_PROTOCOL" % n), p("SERIAL%d_BAUD" % n)
            if proto is None:
                continue
            where = {0: "USB", 1: "TELEM1", 2: "TELEM2", 3: "GPS1",
                     4: "GPS2/serial4", 5: "serial5"}.get(n, "serial%d" % n)
            print("  SERIAL%d        %-8s protocol %s  baud %s" %
                  (n, where, fmt(proto), fmt(baud)))

        # --------------------------------------------------------- battery
        hr("battery monitor")
        print("  BATT_MONITOR   %s" % enum(p("BATT_MONITOR"), link.BATT_MONITORS))
        print("  volt pin %s  curr pin %s  volt mult %s  amp/volt %s" % (
            fmt(p("BATT_VOLT_PIN")), fmt(p("BATT_CURR_PIN")),
            fmt(p("BATT_VOLT_MULT")), fmt(p("BATT_AMP_PERVLT"))))
        print("  capacity %s mAh   low %s V   critical %s V" % (
            fmt(p("BATT_CAPACITY")), fmt(p("BATT_LOW_VOLT")), fmt(p("BATT_CRT_VOLT"))))
        if not p("BATT_MONITOR"):
            todo.append("BATT_MONITOR is 0: no voltage or current monitoring, so no "
                        "battery failsafe and no voltage sag compensation.")

        # -------------------------------------------------------------- RC
        hr("RC input")
        for name in ("RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"):
            print("  %-14s %s" % (name, fmt(p(name))))
        print("  FLTMODE_CH     %s" % fmt(p("FLTMODE_CH")))
        print()
        print("  ch   min   max   trim  rev")
        for n in range(1, 9):
            lo, hi = p("RC%d_MIN" % n), p("RC%d_MAX" % n)
            if lo is None:
                continue
            rev = p("RC%d_REV" % n)
            if rev is None:
                rev = p("RC%d_REVERSED" % n)
            print("  %-4d %-5s %-5s %-5s %s" % (n, fmt(lo), fmt(hi),
                                                fmt(p("RC%d_TRIM" % n)), fmt(rev)))
        default_range = all(p("RC%d_MIN" % n) in (1100, 1000, None) and
                            p("RC%d_MAX" % n) in (1900, 2000, None) for n in (1, 2, 3, 4))
        if default_range:
            todo.append("RC endpoints are at firmware defaults -- the radio has never "
                        "been calibrated on this board. Run ./calibrate_radio.py.")
        print()
        print("  flight modes   %s" % "  ".join(
            "%d:%s" % (n, fmt(p("FLTMODE%d" % n))) for n in range(1, 7)))

        # ------------------------------------------------------- failsafes
        hr("failsafes and arming")
        for name in ("ARMING_CHECK", "ARMING_REQUIRE", "FS_THR_ENABLE", "FS_THR_VALUE",
                     "FS_GCS_ENABLE", "FS_EKF_ACTION", "FS_EKF_THRESH", "FS_CRASH_CHECK",
                     "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT", "FS_OPTIONS",
                     "AHRS_EKF_TYPE", "EK2_ENABLE", "EK3_ENABLE",
                     "BRD_SAFETYENABLE", "BRD_SAFETY_DEFLT", "BRD_SAFETY_MASK",
                     "AHRS_ORIENTATION", "AHRS_TRIM_X", "AHRS_TRIM_Y",
                     "LOG_BITMASK", "INS_GYRO_FILTER", "SCHED_LOOP_RATE"):
            if p(name) is not None:
                print("  %-16s %s" % (name, fmt(p(name))))

    # ------------------------------------------------------------ live state
    hr("live telemetry (%.0fs)" % args.seconds)
    link.request_streams(master)
    latest, counts, texts = link.collect(master, seconds=args.seconds)

    if not any(counts.get(name) for name in link.STREAMED_MSGS):
        print("  NOTHING STREAMING -- the main loop is not running.")
        todo.append("No telemetry streams at all. On this board that usually means a "
                    "firmware fault or a board held in reset, not a sensor problem.")
    else:
        print("  messages seen: %s" % ", ".join(
            "%s x%d" % (k, v) for k, v in sorted(counts.items()) if k != "STATUSTEXT"))

    hb = latest.get("HEARTBEAT")
    if hb is not None:
        armed = bool(hb.base_mode & 128)
        print("  mode           %s   %s" % (link.copter_mode_name(hb.custom_mode),
                                            "ARMED" if armed else "disarmed"))

    sys_status = latest.get("SYS_STATUS")
    if sys_status is not None:
        print()
        print("  sensor                              present enabled healthy")
        for name, present, enabled, healthy in link.sensor_table(sys_status):
            flag = "" if (present and enabled and healthy) or not enabled else "   <-- UNHEALTHY"
            print("  %-35s   %s      %s       %s%s" % (
                name, "y" if present else ".", "y" if enabled else ".",
                "y" if healthy else "n", flag))
        print("  battery %.2f V  current %.2f A  load %.1f%%" % (
            sys_status.voltage_battery / 1000.0,
            sys_status.current_battery / 100.0, sys_status.load / 10.0))

    gps = latest.get("GPS_RAW_INT")
    if gps is not None:
        print("  GPS            %s, %d sats, HDOP %.2f" % (
            enum(gps.fix_type, link.GPS_FIX_TYPES), gps.satellites_visible,
            gps.eph / 100.0))

    rc = latest.get("RC_CHANNELS")
    if rc is not None:
        vals = [getattr(rc, "chan%d_raw" % n) for n in range(1, 9)]
        print("  RC in          %s  (rssi %s)" %
              (" ".join("%4d" % v for v in vals), rc.rssi))
        if all(v == 0 for v in vals):
            todo.append("No RC input: every channel reads 0. Nothing is decoding on "
                        "RCIN -- check the receiver lead, and that the receiver has "
                        "power (the Cube powers RCIN from the flight controller, but "
                        "a receiver on the servo rail needs the battery).")
    servo = latest.get("SERVO_OUTPUT_RAW")
    if servo is not None:
        vals = [getattr(servo, "servo%d_raw" % n) for n in range(1, 9)]
        print("  servo out      %s" % " ".join("%4d" % v for v in vals))

    vib = latest.get("VIBRATION")
    if vib is not None:
        print("  vibration      x %.1f  y %.1f  z %.1f m/s/s   clips %d/%d/%d" % (
            vib.vibration_x, vib.vibration_y, vib.vibration_z,
            vib.clipping_0, vib.clipping_1, vib.clipping_2))

    baros = sum(1 for k in ("SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3")
                if counts.get(k))
    imus = sum(1 for k in ("RAW_IMU", "SCALED_IMU2", "SCALED_IMU3") if counts.get(k))
    print("  live count     %d IMU stream(s), %d barometer stream(s)" % (imus, baros))

    # --------------------------------------------------------------- prearm
    prearm = [t for t in texts if t.startswith(("PreArm", "Arm", "EKF", "AHRS"))]
    if not prearm:
        prearm = link.prearm_messages(master, seconds=6.0)
    if prearm:
        hr("the board's own refusals")
        for text in dict.fromkeys(prearm):
            print("  %s" % text)

    # ------------------------------------------------------------- checklist
    hr("what still needs doing")
    if not todo:
        print("  nothing outstanding that this tool can see.")
    for n, item in enumerate(dict.fromkeys(todo), 1):
        print("  %d. %s" % (n, item))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
