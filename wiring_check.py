#!/usr/bin/env python3
"""Check the airframe's wiring and output setup from the board's own point of view.

    ./wiring_check.py              # everything that works on USB power
    ./wiring_check.py --seconds 10 # longer sample for marginal links

This is the tool to run after rewiring, and the one that answers "is it all
connected right" without a battery and without props. It does not calibrate
anything and writes nothing.

What it can and cannot see, stated plainly because the distinction is the
whole value of the tool:

  CAN see, on USB power alone -- which sensors enumerated and on which bus,
  whether the I2C bus is erroring, whether anything is decoding on RCIN and
  whether it jitters like a live link, which outputs the mixer claims as
  motors, what the 5V rail reads, whether the safety switch is inhibiting
  outputs, whether the GPS serial lead is actually delivering a GPS.

  CANNOT see without the flight battery -- ESC wiring order, motor rotation
  direction, the power module's voltage and current scaling, or anything on
  the servo rail. The Pixhawk powers its own logic and the RCIN port from
  USB, but MAIN OUT carries no power from USB at all: ESC signal grounds are
  referenced through the board, and the ESCs themselves are dead. Those
  checks are listed at the end as "needs the battery", in the order to do
  them.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402

PASS, FAIL, WARN, PEND = "ok", "FAIL", "warn", "pending"
MARK = {PASS: "  ok  ", FAIL: " FAIL ", WARN: " warn ", PEND: "pend. "}


class Report:
    def __init__(self):
        self.rows = []

    def add(self, state, what, detail="", fix=""):
        self.rows.append((state, what, detail, fix))
        print("[%s] %-28s %s" % (MARK[state], what, detail))
        if fix and state in (FAIL, WARN, PEND):
            for chunk in fix.split("\n"):
                print("       %s" % chunk)

    def counts(self, state):
        return [r for r in self.rows if r[0] == state]


def section(title):
    print()
    print("-- %s %s" % (title, "-" * max(0, 66 - len(title))))


# ArduPilot's quad-X layout. Output n drives Motor n; the physical position
# and rotation are fixed by the mixer, not by choice.
QUAD_X_LAYOUT = [
    (1, "front-right", "CCW"),
    (2, "rear-left", "CCW"),
    (3, "front-left", "CW"),
    (4, "rear-right", "CW"),
]

# MAV_CMD_DO_MOTOR_TEST takes a TEST SEQUENCE number, not a motor number:
# ArduPilot walks clockwise round the airframe. Printing the translation is
# the difference between a correctly wired quad looking wired and looking
# scrambled.
QUAD_X_TEST_ORDER = [(1, 1, "front-right"), (2, 4, "rear-right"),
                     (3, 2, "rear-left"), (4, 3, "front-left")]


def main():
    ap = argparse.ArgumentParser(description="wiring and output check (no battery needed)")
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2

    rep = Report()

    banner = link.parse_banner(link.request_banner(master))
    params = link.fetch_params(master)
    params.pop("_count", None)

    def p(*names):
        """First of these parameter names that the firmware actually has."""
        for name in names:
            if name in params:
                return params[name]
        return None

    link.request_streams(master, rate_hz=10)
    # HWSTATUS carries the 5V rail and the cumulative I2C error count, and is
    # not in the ALL stream on every build, so ask for it by id.
    sys_id, comp = link.target(master)
    for msg_id in (link.mavutil.mavlink.MAVLINK_MSG_ID_HWSTATUS,
                   link.mavutil.mavlink.MAVLINK_MSG_ID_POWER_STATUS):
        master.mav.command_long_send(
            sys_id, comp, link.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, 200000, 0, 0, 0, 0, 0)

    first, _, _ = link.collect(master, seconds=1.5)
    latest, counts, texts = link.collect(master, seconds=args.seconds)
    for key, value in first.items():
        latest.setdefault(key, value)

    # ------------------------------------------------------------ the board
    section("board and firmware")
    rep.add(PASS, "firmware", "%s %s (%s) on %s" % (
        banner.get("vehicle", "?"), banner.get("version", "?"),
        banner.get("fw_hash", "?"), banner.get("board_id", "?")))

    # -------------------------------------------------------------- power
    section("power rails")
    power = latest.get("POWER_STATUS")
    hwstat = latest.get("HWSTATUS")

    # The 5V logic rail. POWER_STATUS.Vcc is the measurement every build
    # carries; HWSTATUS also had it but was dropped in later releases, so it
    # is treated as a bonus and never required.
    vcc = None
    if power is not None:
        vcc = power.Vcc / 1000.0
    elif hwstat is not None:
        vcc = hwstat.Vcc / 1000.0
    if vcc is None:
        rep.add(WARN, "5V logic rail", "not reported")
    else:
        # ArduPilot's own pre-arm check fails outside 4.3-5.8 V. Healthy is
        # 4.9-5.4; between 4.3 and 4.8 the board runs but the margin is gone,
        # and a sag under motor load is what trips a brownout in flight.
        state = PASS if 4.8 <= vcc <= 5.4 else (WARN if 4.3 <= vcc < 4.8 else FAIL)
        rep.add(state, "5V logic rail", "%.2f V" % vcc,
                "Low. On USB this is the cable or the hub: a dock or a long\n"
                "charge-oriented cable drops a few hundred mV, and 4.5 V leaves\n"
                "nothing for the servo-rail peripherals. Plug straight into the\n"
                "Mac and re-run before reading anything into it. Under 4.3 V\n"
                "ArduPilot refuses to arm (\'Check Board Voltage\')."
                if state != PASS else "")

    if hwstat is not None:
        rep.add(PASS if hwstat.I2Cerr == 0 else WARN, "I2C error count",
                "%d" % hwstat.I2Cerr,
                "Errors accumulate when an I2C device is marginal: a long or\n"
                "unshielded GPS/compass lead, a loose 4-pin plug, or two devices\n"
                "at the same address. Re-run and see whether it climbs."
                if hwstat.I2Cerr else "")

    if power is not None:
        flags = power.flags
        names = []
        for bit, name in ((1, "brick valid"), (2, "servo rail valid"),
                          (4, "USB connected"), (8, "periph overcurrent"),
                          (16, "hipower overcurrent"), (32, "changed")):
            if flags & bit:
                names.append(name)
        rep.add(PASS, "power status flags", "0x%02x: %s" % (flags, ", ".join(names) or "none"),
                "flags 0 while plainly running on USB means this board does not\n"
                "wire up the VBUS/brick sense lines -- common on clones. It makes\n"
                "the flags uninformative here, not wrong." if flags == 0 else "")

        vservo = power.Vservo / 1000.0
        if vservo < 0.5:
            rep.add(PEND, "servo rail", "%.2f V" % vservo,
                    "0 V with no battery is the textbook reading: MAIN OUT takes\n"
                    "no power from USB. Expect ~5 V from an ESC BEC once the pack\n"
                    "is in.")
        elif vcc and abs(vservo - vcc) < 0.15:
            rep.add(WARN, "servo rail", "%.2f V, tracking the logic rail" % vservo,
                    "The servo rail is live with the battery OUT and sits within\n"
                    "150 mV of the 5V logic rail, so it is being fed from the\n"
                    "board, not from an ESC. Many clones bridge VDD_SERVO to the\n"
                    "5V rail instead of isolating it. Consequence: an ESC BEC and\n"
                    "the board's own 5V supply end up in parallel once the pack\n"
                    "goes in. It flies, but do not also feed the rail from a\n"
                    "second BEC, and treat a receiver browning out under load as\n"
                    "this, not as an RC fault.")
        else:
            rep.add(PASS, "servo rail", "%.2f V" % vservo)

        if flags & 8 or flags & 16:
            rep.add(FAIL, "peripheral overcurrent", "flagged",
                    "Something on the 5V peripheral side draws too much -- usually\n"
                    "a shorted GPS or telemetry lead. Unplug peripherals one at a\n"
                    "time and re-run.")

    # The IO coprocessor matters more here than it looks: on this board MAIN
    # OUT 1-8 and the RCIN decoder both live on it, so RC input and motor
    # output share one failure mode that has nothing to do with either lead.
    for line in banner.get("other", []):
        if line.startswith("IOMCU"):
            rep.add(PASS, "IO coprocessor", line)
        elif line.startswith("RCOut"):
            rep.add(PASS, "output channels", line)
        elif line.startswith("IMU0"):
            rep.add(PASS, "IMU sampling", line)

    # --------------------------------------------------------- safety switch
    section("safety and arming inhibits")
    sys_status = latest.get("SYS_STATUS")
    prearm = [t for t in texts if t.startswith(("PreArm", "Arm"))] or \
        link.prearm_messages(master, seconds=6.0)
    safety_engaged = any("safety switch" in t.lower() for t in prearm)
    outputs_enabled = False
    if sys_status is not None:
        outputs_enabled = bool(sys_status.onboard_control_sensors_enabled & 0x8000)
    if safety_engaged or not outputs_enabled:
        rep.add(PEND, "safety switch", "engaged -- outputs inhibited",
                "Correct and desirable for bench work: no output can move.\n"
                "Press and hold the safety button until the LED stops flashing\n"
                "to enable outputs, and only with props off. BRD_SAFETY_DEFLT 1\n"
                "re-engages it at every boot, which is the right setting.")
    else:
        rep.add(WARN, "safety switch", "disengaged -- outputs LIVE",
                "Outputs can move. Props off, or press the safety switch to\n"
                "re-engage it before touching anything.")
    rep.add(PASS, "servo output now", " ".join(
        "%d" % getattr(latest["SERVO_OUTPUT_RAW"], "servo%d_raw" % n)
        for n in range(1, 9)) if latest.get("SERVO_OUTPUT_RAW") else "no SERVO_OUTPUT_RAW")

    # ------------------------------------------------------------- IMUs
    section("IMUs")
    imu_ids = [(n, p("INS_ACC%s_ID" % s)) for n, s in ((1, ""), (2, "2"), (3, "3"))]
    streaming = [k for k in ("RAW_IMU", "SCALED_IMU2", "SCALED_IMU3") if counts.get(k)]
    detected = [(n, v) for n, v in imu_ids if v]
    for n, devid in detected:
        rep.add(PASS, "IMU %d" % n,
                link.devid_str(link.decode_devid(devid, link.IMU_DEVTYPES)))
    if not detected:
        rep.add(FAIL, "IMUs", "none reported",
                "No IMU means no flight. Reseat the board, and check the\n"
                "firmware target matches the hardware.")
    if streaming and len(streaming) != len(detected):
        # SCALED_IMU2/3 are published whether or not a second IMU exists, so a
        # stream count above the device-id count is the firmware padding, not
        # a sensor the parameter table missed.
        rep.add(PASS, "IMU streams", "%d streaming (%s), %d with device IDs" % (
            len(streaming), ", ".join(streaming), len(detected)))
    if len(detected) == 1:
        rep.add(WARN, "IMU redundancy", "single IMU",
                "One IMU is enough to fly but has no fallback: an IMU fault in\n"
                "flight is unrecoverable rather than a switchover. Normal for a\n"
                "clone board that populates only the MPU6000.")

    imu = latest.get("RAW_IMU")
    if imu is not None:
        # RAW_IMU on ArduPilot is mG for accel and mrad/s for gyro.
        accel = (imu.xacc ** 2 + imu.yacc ** 2 + imu.zacc ** 2) ** 0.5 / 1000.0 * 9.80665
        gyro = max(abs(imu.xgyro), abs(imu.ygyro), abs(imu.zgyro)) / 1000.0
        rep.add(PASS if 8.5 <= accel <= 11.0 else FAIL, "accel magnitude at rest",
                "%.2f m/s/s" % accel,
                "Should be ~9.8 (one g) whatever attitude the board is in.\n"
                "A long way off means a dead axis, not a calibration problem --\n"
                "calibration cannot change the magnitude by this much."
                if not 8.5 <= accel <= 11.0 else "")
        rep.add(PASS if gyro < 0.05 else WARN, "gyro at rest",
                "max %.3f rad/s" % gyro,
                "The aircraft is moving, or an axis is faulty. Note that a\n"
                "reading of exactly 0 on a still board is NOT a stuck axis:\n"
                "one LSB is 0.001 rad/s and a healthy MPU6000 is quieter than\n"
                "that. Test a gyro by rotating it -- ./check_imu.py."
                if gyro >= 0.05 else "")
    vib = latest.get("VIBRATION")
    if vib is not None:
        worst = max(vib.vibration_x, vib.vibration_y, vib.vibration_z)
        rep.add(PASS if worst < 5 else WARN, "vibration at rest",
                "%.1f m/s/s, clips %d/%d/%d" % (worst, vib.clipping_0,
                                                vib.clipping_1, vib.clipping_2))

    # --------------------------------------------------------- barometer
    section("barometer")
    baro = latest.get("SCALED_PRESSURE")
    baro_count = sum(1 for k in ("SCALED_PRESSURE", "SCALED_PRESSURE2",
                                 "SCALED_PRESSURE3") if counts.get(k))
    if baro is None:
        rep.add(FAIL, "barometer", "no SCALED_PRESSURE",
                "Without a barometer there is no altitude: ALT_HOLD, LOITER,\n"
                "AUTO and RTL are all unavailable.")
    else:
        hpa = baro.press_abs
        ok = 800 <= hpa <= 1100
        rep.add(PASS if ok else FAIL, "barometer", "%.1f hPa, %.1f C (%d sensor%s)" % (
            hpa, baro.temperature / 100.0, baro_count, "" if baro_count == 1 else "s"),
            "Sea-level pressure is ~1013 hPa and a sane reading anywhere on\n"
            "earth is 800-1100. Outside that the sensor is not being read." if not ok else "")

    # ---------------------------------------------------------- compasses
    section("compasses")
    mags = []
    for n, suffix in ((1, ""), (2, "2"), (3, "3")):
        devid = p("COMPASS_DEV_ID%s" % suffix, "COMPASS_PRIO%d_ID" % n)
        info = link.decode_devid(devid or 0, link.COMPASS_DEVTYPES)
        if info is None:
            continue
        ext = p("COMPASS_EXTERNAL" if n == 1 else "COMPASS_EXTERN%d" % n)
        use = p("COMPASS_USE" if n == 1 else "COMPASS_USE%d" % n)
        ofs = [p("COMPASS_OFS%s_%s" % (suffix, ax)) or 0 for ax in "XYZ"]
        mag = sum(v * v for v in ofs) ** 0.5
        mags.append((n, info, ext, use, mag))
        rep.add(PASS, "compass %d" % n, "%s  USE=%g  EXTERNAL=%g  offsets |%.0f|" % (
            link.devid_str(info), use if use is not None else -1,
            ext if ext is not None else -1, mag))

    if not mags:
        rep.add(FAIL, "compasses", "none detected",
                "With no magnetometer, heading comes from the GPS only.\n"
                "LOITER, RTL and AUTO will not work.")
    else:
        i2c = [m for m in mags if m[1]["bus_type"] == "I2C"]
        spi = [m for m in mags if m[1]["bus_type"] == "SPI"]
        if i2c:
            rep.add(PASS, "external compass lead", "%d on I2C" % len(i2c),
                    "")
        else:
            rep.add(WARN, "external compass lead", "no compass on I2C",
                    "The GPS mast's magnetometer reaches the board over I2C. If a\n"
                    "GPS/compass module is plugged in and nothing shows on I2C,\n"
                    "the 4-pin I2C lead is the suspect, not the compass.")
        if spi and i2c:
            rep.add(PASS, "compass redundancy",
                    "%d onboard (SPI) + %d external (I2C)" % (len(spi), len(i2c)))
        for n, info, ext, use, mag in i2c:
            if not ext:
                rep.add(WARN, "compass %d EXTERNAL flag" % n, "0, but probed on I2C",
                        "If this is the mast magnetometer the flag is wrong, and the\n"
                        "automatic orientation fit will treat it as board-aligned.\n"
                        "./which_compass.py settles which sensor is which by\n"
                        "rotating the mast and the airframe separately.")
        if all(m[4] == 0 for m in mags):
            rep.add(PEND, "compass calibration", "never calibrated",
                    "Offsets are all zero. ./calibrate_compass.py, with the\n"
                    "aircraft fully assembled -- hard iron comes from the ESCs,\n"
                    "wiring and battery, so a bare board measures the wrong thing.")

    # ---------------------------------------------------------------- GPS
    section("GPS")
    gps_type = p("GPS1_TYPE", "GPS_TYPE")
    gps2_type = p("GPS2_TYPE", "GPS_TYPE2")
    rep.add(PASS, "GPS1_TYPE", link.GPS_TYPES.get(int(gps_type or 0), str(gps_type)))
    if gps2_type:
        rep.add(PASS, "GPS2_TYPE", link.GPS_TYPES.get(int(gps2_type), str(gps2_type)))
    for n in (3, 4):
        proto = p("SERIAL%d_PROTOCOL" % n)
        if proto is not None:
            rep.add(PASS if proto == 5 else WARN, "SERIAL%d protocol" % n,
                    "%g%s" % (proto, " (GPS)" if proto == 5 else ""),
                    "Protocol 5 is GPS. The GPS1 port is SERIAL3 on this board;\n"
                    "if the module is plugged into TELEM1/TELEM2 instead, that\n"
                    "port needs protocol 5 -- or move the lead." if proto != 5 else "")
    gps = latest.get("GPS_RAW_INT")
    if gps is None or gps.fix_type == 0:
        has_i2c_mag = any(m[1]["bus_type"] == "I2C" for m in mags)
        rep.add(FAIL, "GPS receiver", "not detected (fix_type 0)",
                ("The compass inside the GPS module IS being read over I2C, so the\n"
                 "module has power and its 4-pin lead is fine -- which narrows this\n"
                 "to the 6-pin GPS serial lead: unplugged, in the wrong port, or\n"
                 "swapped with the I2C one. That is the single most likely fault\n"
                 "here.\n" if has_i2c_mag else
                 "Nothing is arriving on the GPS serial port.\n") +
                "Check: 6-pin lead into GPS1 (not TELEM), connector fully seated,\n"
                "and that the module's own LED lights. Then ./gps_check.py.\n"
                "Indoors a module will often get no FIX -- but it should still be\n"
                "DETECTED, which is what fix_type 0 says it is not.")
    else:
        ok = gps.fix_type >= 3
        rep.add(PASS if ok else PEND, "GPS receiver", "%s, %d sats, HDOP %.2f" % (
            link.GPS_FIX_TYPES.get(gps.fix_type, gps.fix_type),
            gps.satellites_visible, gps.eph / 100.0),
            "Detected but no 3D fix. Normal indoors -- take it outside with a\n"
            "clear sky view before trusting LOITER or RTL." if not ok else "")

    # ----------------------------------------------------------- RC input
    section("RC input")
    protocols = p("RC_PROTOCOLS")
    if protocols is not None:
        rep.add(PASS, "RC_PROTOCOLS", "%g%s" % (
            protocols, " (all protocols enabled)" if int(protocols) == 1 else ""))
    # Sample RC over a window: a live decoded channel jitters a few us, an
    # idle default is bit-identical. That is the only way to tell them apart.
    rc_min, rc_max, rc_samples = {}, {}, 0
    deadline = __import__("time").time() + min(4.0, args.seconds)
    while __import__("time").time() < deadline:
        msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
        if msg is None:
            continue
        rc_samples += 1
        for n in range(1, 17):
            value = getattr(msg, "chan%d_raw" % n, 0)
            rc_min[n] = min(rc_min.get(n, value), value)
            rc_max[n] = max(rc_max.get(n, value), value)
    if not rc_samples:
        rep.add(FAIL, "RC stream", "no RC_CHANNELS at all")
    else:
        live_channels = [n for n in rc_min if rc_min[n] not in (0, 65535)]
        if not live_channels:
            rep.add(FAIL, "RC receiver", "all channels 0 over %d samples" % rc_samples,
                    "Nothing is decoding on RCIN. For an FS-iA6B on SBUS:\n"
                    "  - SBUS output of the receiver, not the servo channels\n"
                    "  - into the board's RCIN / SBUS-in pin (RCIN is powered\n"
                    "    from the board, so it works on USB alone -- a receiver\n"
                    "    LED that stays dark means the lead or its polarity)\n"
                    "  - signal wire to signal, not swapped with ground\n"
                    "  - the FS-i6 must be bound and switched ON\n"
                    "Individual PWM leads -- what the APM 2.8 used -- do NOT work\n"
                    "here: this board has one RCIN input, not eight.")
        else:
            jittery = [n for n in live_channels if rc_max[n] != rc_min[n]]
            rep.add(PASS, "RC receiver", "%d channels decoding, %d jittering" % (
                len(live_channels), len(jittery)))
            print("       ch  %s" % "  ".join("%4d" % n for n in sorted(live_channels)[:8]))
            print("       now %s" % "  ".join("%4d" % rc_max[n]
                                              for n in sorted(live_channels)[:8]))
            if not jittery:
                rep.add(WARN, "RC liveness", "no channel moved at all",
                        "Every channel is bit-identical across samples. A real\n"
                        "decoded link wobbles a few us; a held failsafe frame does\n"
                        "not. Move a stick and re-run.")
        ch = int(p("FLTMODE_CH") or 5)
        rep.add(PEND, "mode channel", "FLTMODE_CH = %d" % ch,
                "On the FS-i6 only SwC is a 3-position switch -- SwA, SwB and SwD\n"
                "are 2-position. Assign SwC to channel %d in Functions setup ->\n"
                "Aux. channels, and save with a LONG press of CANCEL (a short\n"
                "press discards, which is why assignments 'do not stick').\n"
                "Verify with ./mode_monitor.py." % ch)

    # -------------------------------------------------------- motor outputs
    section("motor outputs and ESCs")
    fclass, ftype = p("FRAME_CLASS"), p("FRAME_TYPE")
    rep.add(PASS if fclass else FAIL, "FRAME_CLASS",
            link.FRAME_CLASSES.get(int(fclass or 0), str(fclass)),
            "0 means no mixer exists: every output stays Disabled, nothing is a\n"
            "motor, and the board cannot arm. ./preload_params.py frame-quad-x"
            if not fclass else "")
    rep.add(PASS, "FRAME_TYPE", link.FRAME_TYPES.get(int(ftype or 0), str(ftype)))

    motors = []
    for n in range(1, 15):
        func = p("SERVO%d_FUNCTION" % n)
        if func is None:
            continue
        name = link.SERVO_FUNCTIONS.get(int(func), "function %g" % func)
        if 33 <= int(func) <= 44:
            motors.append((n, int(func) - 32))
    if motors:
        rep.add(PASS, "outputs claimed as motors",
                ", ".join("MAIN%d=Motor%d" % (out, m) for out, m in motors))
    else:
        rep.add(FAIL, "outputs claimed as motors", "none",
                "No output is assigned to a motor, so a motor test cannot run and\n"
                "the ESCs will never see a signal. Set FRAME_CLASS first -- the\n"
                "mixer assigns these automatically, you do not set SERVOn_FUNCTION\n"
                "by hand on a multirotor.")

    pwm_type = int(p("MOT_PWM_TYPE") or 0)
    rep.add(PASS, "MOT_PWM_TYPE", link.MOT_PWM_TYPES.get(pwm_type, str(pwm_type)))
    rep.add(PASS, "output rate / limits", "RC_SPEED %g Hz, MOT_PWM_MIN/MAX %g/%g" % (
        p("RC_SPEED") or 0, p("MOT_PWM_MIN") or 0, p("MOT_PWM_MAX") or 0),
        "")
    rep.add(PASS, "spin thresholds", "MOT_SPIN_ARM %g, MOT_SPIN_MIN %g" % (
        p("MOT_SPIN_ARM") or 0, p("MOT_SPIN_MIN") or 0))

    print()
    print("       quad X layout the mixer will use, once FRAME_CLASS is set:")
    for motor, where, spin in QUAD_X_LAYOUT:
        print("         MAIN OUT %d -> Motor %d  %-12s %s" % (motor, motor, where, spin))
    print("       Motor numbering is the mixer's, and is NOT the order a motor")
    print("       test walks: DO_MOTOR_TEST takes a test SEQUENCE number and goes")
    print("       clockwise from the front-right arm --")
    for seq, motor, where in QUAD_X_TEST_ORDER:
        print("         sequence %d -> Motor %d (%s)" % (seq, motor, where))

    # ------------------------------------------------------- battery monitor
    section("battery monitor")
    monitor = int(p("BATT_MONITOR") or 0)
    rep.add(PASS if monitor else PEND, "BATT_MONITOR",
            link.BATT_MONITORS.get(monitor, str(monitor)),
            "0 means no voltage or current sensing: no battery failsafe, and no\n"
            "thrust compensation as the pack sags. ./preload_params.py power-module"
            if not monitor else "")
    if monitor:
        rep.add(PASS, "analog pins / scaling",
                "volt pin %g, curr pin %g, mult %g, A/V %g" % (
                    p("BATT_VOLT_PIN") or -1, p("BATT_CURR_PIN") or -1,
                    p("BATT_VOLT_MULT") or 0, p("BATT_AMP_PERVLT") or 0))
    batt = latest.get("SYS_STATUS")
    if batt is not None:
        volts = batt.voltage_battery / 1000.0
        state, message = link.battery_verdict(volts, p("BATT_LOW_VOLT"))
        rep.add({"ok": PASS, "low": WARN, "absent": PEND}.get(state, WARN),
                "pack voltage", message,
                "Expected with the pack out. Once it is in, compare this against\n"
                "a meter BEFORE trusting the battery failsafe: an uncalibrated\n"
                "divider reads low, and a low reading triggers RTL in flight."
                if state != "ok" else "")

    # ----------------------------------------------------------- telemetry
    section("telemetry and logging")
    for n, where in ((1, "TELEM1"), (2, "TELEM2")):
        proto, baud = p("SERIAL%d_PROTOCOL" % n), p("SERIAL%d_BAUD" % n)
        if proto is not None:
            rep.add(PASS, where, "protocol %g at %g00 baud" % (proto, baud or 0))
    log_backend = p("LOG_BACKEND_TYPE")
    logging_ok = sys_status is not None and \
        bool(sys_status.onboard_control_sensors_health & 0x1000000)
    rep.add(PASS if logging_ok else WARN, "logging",
            "LOG_BACKEND_TYPE %g, health %s" % (log_backend or 0,
                                                "ok" if logging_ok else "not ok"),
            "No dataflash logging usually means no SD card, or one the board\n"
            "cannot mount. Without logs a bad first flight teaches nothing."
            if not logging_ok else "")

    # ------------------------------------------------------------- verdict
    section("verdict")
    for state, label in ((FAIL, "blocking"), (WARN, "worth a look"), (PEND, "pending")):
        rows = rep.counts(state)
        if rows:
            print("  %s (%d):" % (label, len(rows)))
            for _, what, detail, _ in rows:
                print("    - %s: %s" % (what, detail))
    if not rep.counts(FAIL):
        print("  nothing blocking on the USB-only checks.")

    print()
    print("  needs the flight battery (props OFF, in this order):")
    print("    1. ./preload_params.py frame-quad-x power-module ...  -- configure first")
    print("    2. ESC calibration -- all four at once, throttle high at power-up.")
    print("       Props off and the aircraft held down. Not scripted here on")
    print("       purpose: it commands full throttle by design.")
    print("    3. ./motor_test.py --seq 1..4 -- order and rotation, one at a time")
    print("    4. compare pack voltage against a meter, then set the failsafe")
    print("    5. ./preflight.py -- the whole set, blockers vs warnings")
    print()
    return 1 if rep.counts(FAIL) else 0


if __name__ == "__main__":
    sys.exit(link.run(main))
