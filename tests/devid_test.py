#!/usr/bin/env python3
"""Self-tests for the device-id decoder and the profile files.

    ./.venv/bin/python tests/devid_test.py

The devid decoder earns a test because it is transcribed from ArduPilot's
enums rather than imported, and a wrong entry produces a confident, wrong
sensor name -- the first version of the table reported this aircraft's
IST8310 as an ICM20948. The values below are real readings from the board.
"""

import glob
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "lib"))

import cubelink as link

FAILURES = []


def check(label, got, want):
    if got != want:
        FAILURES.append("%s: got %r, want %r" % (label, got, want))
        print("FAIL %s: got %r, want %r" % (label, got, want))
    else:
        print("ok   %s" % label)


def main():
    # Real values read from this airframe, 2026-10-02.
    mag = link.decode_devid(658945, link.COMPASS_DEVTYPES)
    check("compass devid 658945 name", mag["name"], "IST8310")
    check("compass devid 658945 bus", (mag["bus_type"], mag["bus"], mag["address"]),
          ("I2C", 0, 0x0E))

    imu = link.decode_devid(1245450, link.IMU_DEVTYPES)
    check("imu devid 1245450 name", imu["name"], "MPU6000 (accel)")
    check("imu devid 1245450 bus", imu["bus_type"], "SPI")

    check("devid 0 is no device", link.decode_devid(0, link.COMPASS_DEVTYPES), None)

    # Bit packing: bus_type 3 bits, bus 5, address 8, devtype 8.
    packed = 2 | (3 << 3) | (0x77 << 8) | (0x0B << 16)
    got = link.decode_devid(packed, link.BARO_DEVTYPES)
    check("packing round-trip",
          (got["bus_type"], got["bus"], got["address"], got["name"]),
          ("SPI", 3, 0x77, "MS5611"))

    # An unknown devtype must print as a number, never as a guess.
    unknown = link.decode_devid(0xFF0000 | 1, link.COMPASS_DEVTYPES)
    check("unknown devtype is not guessed", "unknown" in unknown["name"], True)

    # Version parsing drives the "should you upgrade" advice.
    check("version 3.6.12", link.version_tuple("3.6.12"), (3, 6, 12))
    check("version 4.7.1 release candidate", link.version_tuple("4.7.1-rc2"), (4, 7, 1))
    check("version junk", link.version_tuple("not a version"), None)

    # Banner parsing, both the NuttX and the ChibiOS spellings.
    nuttx = link.parse_banner([
        "ArduCopter V3.6.12 (cb570c06)", "PX4: 1e7ed30b NuttX: 1472b16c",
        "PX4v3 00280023 31345101 31383230", "Frame: UNKNOWN"])
    check("nuttx banner version", nuttx["version"], "3.6.12")
    check("nuttx banner board", nuttx["board_id"], "PX4v3")
    chibios = link.parse_banner([
        "ArduCopter V4.7.1 (dbe79216)", "ChibiOS: d103983f",
        "fmuv3 00280023 31345101 31383230", "RCOut: PWM:1-14"])
    check("chibios banner version", chibios["version"], "4.7.1")
    check("chibios banner board", chibios["board_id"], "fmuv3")
    check("chibios keeps RCOut as other", "RCOut: PWM:1-14" in chibios["other"], True)

    # Every profile must parse as NAME,VALUE.
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for path in sorted(glob.glob(os.path.join(root, "profiles", "*.params"))):
        rows = 0
        for lineno, line in enumerate(open(path), 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r"[,\s]+", line, maxsplit=1)
            ok = len(parts) == 2
            if ok:
                try:
                    float(parts[1])
                except ValueError:
                    ok = False
            if not ok:
                FAILURES.append("%s:%d bad line %r" % (path, lineno, line))
            rows += 1
        check("profile %s parses (%d values)" % (os.path.basename(path), rows), True, True)

    # --- error handling: the two failures that actually happen in the field ---
    # Verified here rather than by holding the serial port, because testing it
    # against hardware means becoming the second process on the port, which is
    # the very fault being tested.
    class FakeSerialException(Exception):
        pass
    FakeSerialException.__name__ = "SerialException"

    def boom_serial():
        raise FakeSerialException(
            "device reports readiness to read but returned no data "
            "(device disconnected or multiple access on port?)")

    def boom_interrupt():
        raise KeyboardInterrupt()

    def boom_link():
        raise link.LinkError("no such port: /dev/nope")

    def boom_other():
        raise ValueError("a real bug")

    check("run() maps a serial conflict to exit 2", link.run(boom_serial), 2)
    check("run() maps Ctrl-C to exit 130", link.run(boom_interrupt), 130)
    check("run() maps LinkError to exit 2", link.run(boom_link), 2)
    check("run() passes a clean return through", link.run(lambda: 0), 0)
    try:
        link.run(boom_other)
        check("run() re-raises an unrelated exception", False, True)
    except ValueError:
        check("run() re-raises an unrelated exception", True, True)

    # connect() must refuse a port another process already holds. macOS will
    # happily hand out a second descriptor, so this check is the only thing
    # standing between two tools and an unexplained mid-run read failure.
    real_holder, real_resolve = link.port_holder, link.resolve_port
    try:
        link.resolve_port = lambda explicit=None: "/dev/cu.fake"
        link.port_holder = lambda port: [("99999", "QGroundControl")]
        try:
            link.connect(quiet=True)
            check("connect() refuses a held port", False, True)
        except link.LinkError as exc:
            check("connect() refuses a held port", True, True)
            check("the refusal names the holder", "QGroundControl" in str(exc), True)
    finally:
        link.port_holder, link.resolve_port = real_holder, real_resolve

    print()
    if FAILURES:
        print("%d failure(s)" % len(FAILURES))
        return 1
    print("all tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
