"""Shared MAVLink plumbing for the Pixhawk 2.1 Cube (fmuv3) running ArduCopter.

Ported from ~/apm28-tools/lib/apmlink.py. The port-handling and
verified-write machinery carried over unchanged -- those problems are macOS
and ArduPilot problems, not APM problems. Everything version-specific did
not: this board speaks MAVLink 2, implements COMMAND_INT,
SET_MESSAGE_INTERVAL and the onboard MAG_CAL/accel-cal routines, and does
not need the v1 bridge or the blind SR0_* writes that APM 2.8 required.

Two things here are deliberately dynamic rather than hard-coded, because
this board's firmware is a moving target (3.6.12 today, 4.5.x after an
upgrade) and parameter names moved between those releases:

  * the parameter table is fetched whole and then matched by regex, so a
    tool asks "which compasses does this firmware report" instead of
    "what is COMPASS_DEV_ID3".
  * stream requests go through request_streams(), which uses
    SET_MESSAGE_INTERVAL where the firmware advertises it and falls back to
    REQUEST_DATA_STREAM where it does not.
"""

import glob
import importlib.util
import os
import re
import subprocess
import sys
import time


def _reexec_into_venv():
    """Re-run the calling script with the project venv's interpreter.

    Every tool here starts `#!/usr/bin/env python3`, which picks the SYSTEM
    python when a tool is run as ./tool.py -- and the system python has no
    pymavlink, so the tool dies with ModuleNotFoundError before printing
    anything useful. The APM toolkit had the same trap and answered it with a
    note in the README telling people to activate the venv first, which they
    then forget, because a tool that works from one shell and not another is
    indistinguishable from a broken tool.

    So: if pymavlink is missing and the venv exists, hand off to it. One
    exec, no wrapper scripts, and ./tool.py works from any shell.
    """
    if importlib.util.find_spec("pymavlink") is not None:
        return
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    venv_python = os.path.join(root, ".venv", "bin", "python")
    if os.environ.get("CUBELINK_REEXEC"):
        # Already handed off once and pymavlink is still missing; a second
        # exec would loop forever.
        sys.exit("pymavlink is missing from %s.\n"
                 "The venv exists but is incomplete -- re-run ./setup_env.sh"
                 % venv_python)
    if os.path.exists(venv_python):
        os.environ["CUBELINK_REEXEC"] = "1"
        os.execv(venv_python, [venv_python] + sys.argv)
    sys.exit("pymavlink is not installed and there is no venv at %s.\n"
             "Run ./setup_env.sh first." % os.path.join(root, ".venv"))


_reexec_into_venv()

# MAVLink 2 must be chosen before pymavlink's dialect module is imported, and
# importing mavutil is what triggers that. Tools import this module first, so
# this is the right place for it.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402  (must follow the env var above)

# USB CDC ignores the bit rate, but pymavlink insists on a number.
DEFAULT_BAUD = 115200

# /dev/cu.* rather than /dev/tty.*: on macOS the tty.* node blocks on open
# waiting for carrier detect, which a USB CDC board never asserts. A Cube
# enumerates through the STM32's own USB peripheral as cu.usbmodem*; the
# usbserial globs are here for an FTDI/CP2102 telemetry radio on TELEM1.
PORT_GLOBS = [
    "/dev/cu.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
]

# Messages that only exist while the main loop is running. Used to tell a
# board that is merely quiet from one that is not flying its scheduler.
# HEARTBEAT, STATUSTEXT and PARAM_VALUE are excluded on purpose: on APM 2.8
# all three survived a halted main loop, and the same reasoning applies to
# any autopilot that services them from a callback.
STREAMED_MSGS = (
    "ATTITUDE",
    "RAW_IMU",
    "SCALED_IMU2",
    "SCALED_IMU3",
    "SCALED_PRESSURE",
    "GPS_RAW_INT",
    "SYS_STATUS",
    "RC_CHANNELS",
    "SERVO_OUTPUT_RAW",
    "VFR_HUD",
)


class LinkError(RuntimeError):
    pass


# The port most recently opened by this process. Error paths need it to name
# whoever else is holding the device, and by the time a read fails the
# connection object may be unusable.
LAST_PORT = None


# --------------------------------------------------------------------------
# port handling
# --------------------------------------------------------------------------

def find_ports():
    found = []
    for pattern in PORT_GLOBS:
        for path in sorted(glob.glob(pattern)):
            if path not in found:
                found.append(path)
    return found


def resolve_port(explicit=None):
    if explicit:
        if not os.path.exists(explicit):
            raise LinkError("no such port: %s" % explicit)
        return explicit
    found = find_ports()
    if not found:
        raise LinkError(
            "No USB serial port found.\n"
            "  1. Cable -- a charge-only micro-USB cable is the usual cause.\n"
            "  2. Plug straight into the Mac, not through a hub or dock.\n"
            "  3. ls /dev/cu.* to see everything macOS enumerated.\n"
            "  4. A Cube with no firmware sits in the bootloader for 5s and\n"
            "     then disappears from /dev; ./flash_cube.sh --list-ports\n"
            "     catches it in that window."
        )
    if len(found) > 1:
        raise LinkError(
            "Several candidate ports; pick one with --port:\n  " + "\n  ".join(found)
        )
    return found[0]


def port_holder(port):
    """Which processes currently have this serial device open.

    macOS exposes the same USB device as both /dev/cu.X and /dev/tty.X, and a
    program holding either one blocks the other, so both are checked.
    """
    candidates = [port]
    base = os.path.basename(port)
    if base.startswith("cu."):
        candidates.append(os.path.join(os.path.dirname(port), "tty." + base[3:]))
    elif base.startswith("tty."):
        candidates.append(os.path.join(os.path.dirname(port), "cu." + base[4:]))

    holders, seen = [], set()
    for candidate in candidates:
        try:
            result = subprocess.run(["lsof", "-t", "-w", candidate],
                                    capture_output=True, text=True, timeout=5)
        except (FileNotFoundError, subprocess.SubprocessError):
            return []
        for pid in result.stdout.split():
            pid = pid.strip()
            if not pid or pid in seen:
                continue
            seen.add(pid)
            try:
                name = subprocess.run(["ps", "-p", pid, "-o", "comm="],
                                      capture_output=True, text=True,
                                      timeout=5).stdout.strip()
            except (FileNotFoundError, subprocess.SubprocessError):
                name = "?"
            holders.append((pid, os.path.basename(name) or "?"))
    return holders


def busy_port_message(port):
    holders = port_holder(port)
    lines = ["%s is already open by another process." % port]
    if holders:
        lines.append("")
        for pid, name in holders:
            lines.append("  holding it:  %s  (pid %s)" % (name, pid))
    lines.append("")
    lines.append("Only one process can own a serial port. Common culprits:")
    lines.append("  - QGroundControl. Disconnecting the link is NOT enough: its")
    lines.append("    Comm Links -> AutoConnect settings reopen the port on their")
    lines.append("    own. Quit QGC, or turn those toggles off.")
    lines.append("  - another tool from this directory still running")
    lines.append("  - a stale MAVProxy, screen or minicom session")
    return "\n".join(lines)


def open_serial(port, baud, **kwargs):
    try:
        return mavutil.mavlink_connection(port, baud=baud, **kwargs)
    except Exception as exc:
        text = str(exc).lower()
        if "busy" in text or "errno 16" in text:
            raise LinkError(busy_port_message(port))
        raise LinkError("could not open %s: %s" % (port, exc))


def connect(port=None, baud=DEFAULT_BAUD, timeout=30, quiet=False):
    """Open the link and wait for a heartbeat.

    Unlike the APM, opening this port does not reset the board: the Cube's
    USB stack is part of the running firmware, so a connect is just a
    connect. That cuts both ways -- the boot banner has usually been and gone
    long before you attach, which is why identity comes from an explicit
    DO_SEND_BANNER (see request_banner) rather than from whatever text
    happened to be on the wire.
    """
    port = resolve_port(port)

    # Check for another holder BEFORE opening. macOS does not refuse a second
    # open of /dev/cu.*: both processes get a working file descriptor and then
    # steal each other's bytes, so the failure surfaces much later as
    # "device reports readiness to read but returned no data" from a random
    # read deep inside a tool. Catching it here turns a confusing mid-run
    # traceback into a message naming the process to quit.
    holders = [(pid, name) for pid, name in port_holder(port)
               if pid != str(os.getpid())]
    if holders:
        raise LinkError(busy_port_message(port))

    global LAST_PORT
    LAST_PORT = port
    if not quiet:
        print("link   %s" % port)
    master = open_serial(port, baud)

    deadline = time.time() + timeout
    hb = None
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is not None:
            break
        if not quiet:
            print(".", end="", flush=True)
    if hb is None:
        raise LinkError(
            "No heartbeat in %ds. Either the board has no firmware, or it is\n"
            "sitting in the bootloader, or another program holds the port.\n"
            "%s" % (timeout, busy_port_message(port))
        )

    # ArduPilot reports component 0 in some builds where the wire shows 1.
    # Commands addressed to component 0 are accepted, but MAG_CAL and the
    # calibration routines are cleaner when addressed to the autopilot
    # component explicitly.
    master.cube_comp = master.target_component or 1
    if not quiet:
        print("board  system %d component %d" % (master.target_system, master.cube_comp))
    return master


def target(master):
    return master.target_system, getattr(master, "cube_comp", 1) or 1


# Serial failures that mean "something else is using this port", or "the board
# went away", rather than anything about the aircraft.
_CONFLICT_HINTS = (
    "multiple access",
    "returned no data",
    "device not configured",
    "device disconnected",
    "input/output error",
    "errno 6",
    "errno 16",
    "resource busy",
)


def run(main_fn, interrupt_note=None):
    """Run a tool's main() and turn plumbing failures into useful messages.

    Every tool here ends with `sys.exit(link.run(main))`. Without it, the
    two most common real-world failures -- another process on the serial
    port, and the board being unplugged mid-run -- arrive as a pyserial
    traceback from somewhere deep inside recv_match, which says nothing
    about what to do and looks like a bug in the tool.
    """
    try:
        return main_fn()
    except KeyboardInterrupt:
        print()
        if interrupt_note:
            print(interrupt_note)
        return 130
    except LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    except Exception as exc:                       # noqa: BLE001
        text = str(exc).lower()
        if exc.__class__.__name__ in ("SerialException", "SerialTimeoutException") \
                or any(hint in text for hint in _CONFLICT_HINTS):
            print(file=sys.stderr)
            print("the serial link failed mid-run:", file=sys.stderr)
            print("  %s" % exc, file=sys.stderr)
            print(file=sys.stderr)
            if LAST_PORT:
                print(busy_port_message(LAST_PORT), file=sys.stderr)
            else:
                print("Either another process is using the port, or the board was",
                      file=sys.stderr)
                print("unplugged.", file=sys.stderr)
            print(file=sys.stderr)
            print("If nothing else is holding it, the board itself dropped off:",
                  file=sys.stderr)
            print("replug USB and try again.", file=sys.stderr)
            return 2
        raise


# --------------------------------------------------------------------------
# identity
# --------------------------------------------------------------------------

BANNER_PATTERNS = {
    "firmware": re.compile(r"^(Ardu\w+)\s+V(\d+\.\d+\.\d+\S*)\s*\(([0-9a-f]+)\)"),
    "os": re.compile(r"^(PX4|ChibiOS):\s*(\S+)(?:\s+NuttX:\s*(\S+))?"),
    "board": re.compile(r"^(PX4v\d\S*|[A-Za-z0-9]+)\s+([0-9A-F]{8})\s+([0-9A-F]{8})\s+([0-9A-F]{8})"),
    "frame": re.compile(r"^Frame:\s*(\S+)"),
}


_LAST_HEARTBEAT = {}


def heartbeat(master, interval=1.0):
    """Announce ourselves as a GCS. Call this inside any long polling loop.

    ArduPilot's interactive calibrations -- the six-position accel routine in
    particular -- only drive a channel it regards as a live GCS, and that
    status comes from receiving HEARTBEAT. pymavlink sends none on its own,
    so a tool can issue MAV_CMD_PREFLIGHT_CALIBRATION, get no refusal, and
    then wait forever for a "Place vehicle level" prompt that the board never
    sends because it has nobody to send it to.

    That is exactly what happened here, and it was masked at first because
    APM Planner was open on the same board supplying the heartbeats.

    Rate-limited rather than threaded on purpose: pymavlink's writer is not
    thread-safe, and interleaving a heartbeat into a half-written frame is a
    harder bug than the one being fixed.
    """
    now = time.time()
    key = id(master)
    if now - _LAST_HEARTBEAT.get(key, 0.0) < interval:
        return
    _LAST_HEARTBEAT[key] = now
    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    except Exception:
        pass


def request_banner(master, seconds=4.0):
    """Ask the board to reprint its identity banner, and collect it.

    MAV_CMD_DO_SEND_BANNER (42428) is ArduPilot-specific and is the only
    reliable way to get the firmware version, the OS build and the board ID
    string from a board that booted hours ago. AUTOPILOT_VERSION carries the
    version number but not the board ID, and on 3.6 it does not always
    arrive at all.
    """
    sys_id, comp = target(master)
    master.mav.command_long_send(sys_id, comp, 42428, 0, 0, 0, 0, 0, 0, 0, 0)
    lines, deadline = [], time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        lines.append(statustext(msg))
    return lines


def statustext(msg):
    text = msg.text
    if isinstance(text, (bytes, bytearray)):
        text = bytes(text).decode("ascii", "replace")
    return text.rstrip("\x00").strip()


def parse_banner(lines):
    """Pull identity out of banner lines. Unmatched lines are kept verbatim."""
    out = {"raw": list(lines), "other": []}
    for line in lines:
        m = BANNER_PATTERNS["firmware"].match(line)
        if m:
            out["vehicle"], out["version"], out["fw_hash"] = m.group(1), m.group(2), m.group(3)
            continue
        m = BANNER_PATTERNS["os"].match(line)
        if m:
            out["os"] = line
            continue
        m = BANNER_PATTERNS["board"].match(line)
        # NuttX builds print the board as "PX4v3"; ChibiOS prints the hwdef
        # name, "fmuv3". Both are followed by the STM32's three-word unique
        # id, which is what makes the line unambiguous.
        if m:
            out["board_id"] = m.group(1)
            # The three hex words are the STM32 96-bit unique device ID.
            out["cpu_uid"] = "%s %s %s" % (m.group(2), m.group(3), m.group(4))
            continue
        m = BANNER_PATTERNS["frame"].match(line)
        if m:
            out["frame_banner"] = m.group(1)
            continue
        out["other"].append(line)
    return out


# PX4 board IDs as ArduPilot's NuttX builds print them. The ones worth telling
# apart here are the two that physically fit a 2.4.8-era airframe.
BOARD_IDS = {
    "fmuv2": "FMUv2 -- Pixhawk 1 class, 1 MB usable flash",
    "fmuv3": "FMUv3 -- Pixhawk 1 class with 2 MB flash (incl. clones), or a Cube",
    "fmuv4": "FMUv4 -- Pixracer",
    "Pixhawk1": "Pixhawk 1 (genuine 3DR hwdef), 2 MB flash",
    "CubeBlack": "Hex/ProfiCNC Cube Black",
    "PX4v1": "Pixhawk 1 / FMUv1 (1 MB flash)",
    "PX4v2": "FMUv2 -- Pixhawk 1 / 2.4.x clone, 2 MB flash limited to 1 MB",
    "PX4v3": "FMUv3 -- Pixhawk 2.1 Cube (Black/Green) or a 2 MB Pixhawk 1 clone",
    "PX4v4": "FMUv4 -- Pixracer",
    "PX4v4pro": "FMUv4-PRO -- Drotek Pixhawk 3 Pro",
}


def version_tuple(version):
    """'3.6.12' or '4.5.7-rc1' -> (3, 6, 12). Returns None if unparseable."""
    if not version:
        return None
    m = re.match(r"(\d+)\.(\d+)\.(\d+)", version)
    return tuple(int(g) for g in m.groups()) if m else None


def autopilot_version(master, seconds=4.0):
    """AUTOPILOT_VERSION, or None. Capability flags come from here only."""
    sys_id, comp = target(master)
    master.mav.command_long_send(
        sys_id, comp, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=0.5)
        if msg is not None:
            return msg
    return None


# --------------------------------------------------------------------------
# device IDs
# --------------------------------------------------------------------------

# AP_HAL::Device::DeviceStructure packs the bus into bit fields:
#   bus_type : 3   bus : 5   address : 8   devtype : 8
# so a devid tells you not just which chip, but which bus it was found on --
# which is how a tool can say "this compass is on the external I2C bus"
# without trusting COMPASS_EXTERNAL, a parameter the user can set wrongly.
BUS_TYPES = {0: "unknown", 1: "I2C", 2: "SPI", 3: "CAN/UAVCAN", 4: "SITL", 5: "MSP", 6: "serial"}

# Compass devtypes, from AP_Compass_Backend::DevTypes. These are NOT in
# pymavlink, so they are transcribed here. Getting one wrong is worse than
# printing a number -- an early pass at this file had 0x0A as ICM20948 and
# so reported the GPS mast's IST8310 as a chip that is not on this aircraft.
# Anything not in the table prints as a raw devtype rather than a guess.
COMPASS_DEVTYPES = {
    0x01: "HMC5883 (old probe)",
    0x02: "LSM303D",
    0x04: "AK8963",
    0x05: "BMM150",
    0x06: "LSM9DS1",
    0x07: "HMC5883",
    0x08: "LIS3MDL",
    0x09: "AK09916",
    0x0A: "IST8310",
    0x0B: "ICM20948",
    0x0C: "MMC3416",
    0x0D: "QMC5883L",
    0x0E: "MAG3110",
    0x0F: "SITL",
    0x10: "IST8308",
    0x11: "RM3100",
    0x12: "MSP",
    0x13: "ExternalAHRS",
    0x14: "MMC5983",
    0x15: "QMC5883P",
    0x16: "BMM350",
    0x17: "IIS2MDC",
}

IMU_DEVTYPES = {
    0x09: "BMI160",
    0x10: "L3G4200D",
    0x11: "LSM303D (accel)",
    0x12: "BMA180 (accel)",
    0x13: "MPU6000 (accel)",
    0x16: "MPU9250 (accel)",
    0x17: "IIS328DQ (accel)",
    0x18: "LSM9DS1 (accel)",
    0x21: "MPU6000 (gyro)",
    0x22: "L3GD20 (gyro)",
    0x24: "MPU9250 (gyro)",
    0x25: "I3G4250D (gyro)",
    0x26: "LSM9DS1 (gyro)",
    0x27: "ICM20789",
    0x28: "ICM20689",
    0x29: "BMI055",
    0x2A: "SITL",
    0x2B: "BMI088",
    0x2C: "ICM20948",
    0x2D: "ICM20648",
    0x2E: "ICM20649",
    0x2F: "ICM20602",
    0x30: "ICM20601",
    0x31: "ADIS1647x",
    0x32: "serial IMU",
    0x33: "ICM40609",
    0x34: "ICM42688",
    0x35: "ICM42605",
    0x36: "ICM40605",
    0x37: "IIM42652",
    0x38: "BMI270",
    0x39: "ICM42670",
    0x3A: "ICM45686",
    0x3B: "SCHA63T",
    0x3C: "IIM42653",
}

BARO_DEVTYPES = {
    0x01: "SITL",
    0x02: "BMP085",
    0x03: "BMP280",
    0x04: "BMP388",
    0x05: "DPS280",
    0x06: "DPS310",
    0x07: "FBM320",
    0x08: "ICM20789",
    0x09: "KellerLD",
    0x0A: "LPS2XH",
    0x0B: "MS5611",
    0x0C: "SPL06",
    0x0D: "DroneCAN",
    0x0E: "MSP",
    0x0F: "ICP101XX",
    0x10: "ICP201XX",
    0x11: "MSP (ext)",
    0x12: "BMP390",
    0x13: "BMP581",
    0x14: "SPA06",
    0x15: "AUAV",
}


def decode_devid(devid, table=None):
    """Unpack a device id. Returns a dict; devid 0 means "no device"."""
    devid = int(devid)
    if devid == 0:
        return None
    bus_type = devid & 0x07
    bus = (devid >> 3) & 0x1F
    address = (devid >> 8) & 0xFF
    devtype = (devid >> 16) & 0xFF
    name = (table or {}).get(devtype)
    return {
        "devid": devid,
        "bus_type": BUS_TYPES.get(bus_type, str(bus_type)),
        "bus_type_num": bus_type,
        "bus": bus,
        "address": address,
        "devtype": devtype,
        "name": name or "devtype 0x%02X (unknown)" % devtype,
    }


def devid_str(info):
    if info is None:
        return "none"
    return "%s on %s bus %d addr 0x%02X" % (
        info["name"], info["bus_type"], info["bus"], info["address"])


# --------------------------------------------------------------------------
# parameters
# --------------------------------------------------------------------------

def fetch_params(master, timeout=180, quiet=False):
    """Fetch the whole parameter table, re-requesting whatever goes missing.

    ArduPilot streams ~1000 parameters here and loss on USB is normal, so a
    single pass through PARAM_VALUE is never complete. The protocol gives
    param_index and param_count, so the gaps are knowable: collect by index,
    then ask for the missing indices individually, and keep doing that until
    the table is whole or the overall timeout runs out.

    Two things this has to survive:

      * a board that has just rebooted drops most of the first burst -- one
        run arrived with 123 of 1009 -- so a fixed number of retry rounds is
        not enough; it loops on the remaining gaps instead.
      * a tool that reports a dropped parameter as a parameter the firmware
        does not have sends people chasing renames that never happened. Being
        slow here is much cheaper than being wrong.
    """
    sys_id, comp = target(master)
    by_index, params = {}, {}
    count = None
    hard_deadline = time.time() + timeout

    def absorb(msg):
        nonlocal count
        name = msg.param_id
        if isinstance(name, (bytes, bytearray)):
            name = bytes(name).decode("ascii", "replace")
        name = name.rstrip("\x00")
        if msg.param_count:
            count = msg.param_count
        by_index[msg.param_index] = name
        params[name] = msg.param_value

    def drain(seconds, quiet_after=1.5):
        last_rx = time.time()
        end = time.time() + seconds
        while time.time() < min(end, hard_deadline):
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                if time.time() - last_rx > quiet_after:
                    return
                continue
            absorb(msg)
            last_rx = time.time()
            if count and len(by_index) >= count:
                return

    def progress(note=""):
        if not quiet:
            print("\r  parameters %d%s%s        " % (
                len(by_index), "/%d" % count if count else "", note),
                end="", flush=True)

    # First pass: ask for the lot. Re-ask if the board ignored it entirely,
    # which happens when it is still booting.
    for _ in range(3):
        master.mav.param_request_list_send(sys_id, comp)
        drain(45)
        progress()
        if count:
            break

    # Then close the gaps. ArduPilot answers param_request_read by index, and
    # a batch of a few hundred at a time keeps the serial link from
    # overflowing its own buffers.
    stuck_rounds = 0
    while count and len(by_index) < count and time.time() < hard_deadline:
        missing = [i for i in range(count) if i not in by_index]
        before = len(by_index)
        progress(", filling %d gaps" % len(missing))
        for index in missing[:150]:
            master.mav.param_request_read_send(sys_id, comp, b"", index)
        drain(10)
        if len(by_index) == before:
            stuck_rounds += 1
            # Nothing at all arrived: the board may have stopped answering.
            # Re-issue the full list request once, then give up on a third
            # idle round rather than spinning to the timeout.
            if stuck_rounds == 1:
                master.mav.param_request_list_send(sys_id, comp)
                drain(15)
            elif stuck_rounds >= 3:
                break
        else:
            stuck_rounds = 0

    if not quiet:
        missing = (count - len(by_index)) if count else 0
        print("\r  parameters %d%s%s        " % (
            len(params), "/%d" % count if count else "",
            "  (%d NOT retrieved -- treat 'absent' with suspicion)" % missing
            if missing > 0 else ""))
    params["_count"] = count or len(params)
    return params


def get_param(master, name, timeout=4.0, retries=3):
    """Read one parameter. None if the firmware does not have it."""
    sys_id, comp = target(master)
    for _ in range(retries):
        master.mav.param_request_read_send(sys_id, comp, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, (bytes, bytearray)):
                got = bytes(got).decode("ascii", "replace")
            if got.rstrip("\x00") == name:
                return msg.param_value
    return None


def set_param(master, name, value, timeout=4.0, retries=3, tol=1e-4):
    """Write one parameter and confirm the board echoed the new value.

    ArduPilot silently clamps out-of-range writes and ignores read-only ones,
    so an unverified write is not a write you can claim happened. Returns
    (ok, readback).
    """
    sys_id, comp = target(master)
    for _ in range(retries):
        master.mav.param_set_send(sys_id, comp, name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, (bytes, bytearray)):
                got = bytes(got).decode("ascii", "replace")
            if got.rstrip("\x00") != name:
                continue
            readback = msg.param_value
            scale = max(1.0, abs(float(value)))
            return (abs(readback - float(value)) <= tol * scale, readback)
    return (False, None)


# --------------------------------------------------------------------------
# streams
# --------------------------------------------------------------------------

def request_streams(master, rate_hz=5, use_interval=None):
    """Turn on telemetry.

    On ArduPilot 4.x SET_MESSAGE_INTERVAL is the supported path and
    REQUEST_DATA_STREAM is deprecated; on 3.6 the reverse is true. Both are
    sent, because sending the wrong one costs a rejected COMMAND_ACK and
    sending neither costs an afternoon of "the board is not streaming".
    """
    sys_id, comp = target(master)
    master.mav.request_data_stream_send(
        sys_id, comp, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)
    if use_interval is None:
        use_interval = True
    if not use_interval:
        return
    interval_us = int(1e6 / max(1, rate_hz))
    wanted = [
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    ]
    for msg_id in wanted:
        master.mav.command_long_send(
            sys_id, comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0)


def collect(master, seconds=4.0, types=None):
    """Listen for a while. Returns (latest-by-type, count-by-type, texts)."""
    latest, counts, texts = {}, {}, []
    deadline = time.time() + seconds
    while time.time() < deadline:
        heartbeat(master)
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "BAD_DATA":
            continue
        if kind == "STATUSTEXT":
            texts.append(statustext(msg))
        if types and kind not in types:
            continue
        latest[kind] = msg
        counts[kind] = counts.get(kind, 0) + 1
    return latest, counts, texts


def telemetry_alive(master, seconds=4.0):
    """Is the main loop running? Behavioural test, not a text match."""
    request_streams(master)
    _, counts, _ = collect(master, seconds=seconds)
    return any(counts.get(name) for name in STREAMED_MSGS)


def prearm_messages(master, seconds=8.0):
    """Collect the PreArm / Arm refusal lines the board emits on its own.

    ArduPilot repeats these roughly every 30s while disarmed, and emits the
    whole set immediately after an arm attempt. This only listens -- it never
    tries to arm.
    """
    _, _, texts = collect(master, seconds=seconds)
    out = []
    for text in texts:
        if text.startswith(("PreArm", "Arm", "EKF", "AHRS")) and text not in out:
            out.append(text)
    return out


def reboot(master, hold_in_bootloader=False, wait=True, timeout=40):
    """Reboot the autopilot and, by default, wait for it to come back.

    Needed more often than it looks. ArduPilot builds a driver's parameter
    tree when the driver is constructed at startup, so enabling a backend --
    BATT_MONITOR 4, a GPS type, a serial protocol -- does not make its
    sub-parameters exist until the next boot. Writing BATT_VOLT_MULT before
    that reboot is not a failed write; the parameter genuinely is not there
    yet.

    Returns a fresh connection when wait is set, because the USB device is
    torn down and re-enumerated, often under a different /dev/cu.usbmodem
    name, so the old file descriptor is dead.
    """
    sys_id, comp = target(master)
    port = getattr(master, "address", None) or getattr(master, "device", None)
    master.mav.command_long_send(
        sys_id, comp, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        3 if hold_in_bootloader else 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)
    try:
        master.close()
    except Exception:
        pass
    if not wait:
        return None

    # The device disappears and reappears, usually under a different name.
    # Two traps, both of which produced a confusing traceback before being
    # handled here:
    #
    #   1. macOS keeps the old /dev/cu.* node around for a moment after the
    #      USB device is gone, so reconnecting immediately opens a dead
    #      descriptor and the first read fails with errno 6, "Device not
    #      configured" -- a SerialException, not a LinkError. So wait for the
    #      old node to actually disappear before looking for the new one.
    #   2. the node can appear before the USB CDC endpoint is ready, so the
    #      first connect attempt legitimately fails and simply wants retrying.
    deadline = time.time() + timeout
    while time.time() < deadline and port and os.path.exists(port):
        time.sleep(0.5)

    last = None
    while time.time() < deadline:
        if find_ports():
            try:
                fresh = connect(timeout=8, quiet=True)
                # Give the scheduler a moment: a board still finishing its
                # init drops most of a parameter burst, and the caller's
                # next move is almost always to read parameters.
                time.sleep(3.0)
                return fresh
            except Exception as exc:          # LinkError or a serial error
                last = exc
        time.sleep(1.0)
    raise LinkError("board did not come back within %ds of a reboot (was %s)%s"
                    % (timeout, port, "\n  last attempt: %s" % last if last else ""))


# --------------------------------------------------------------------------
# decoding helpers
# --------------------------------------------------------------------------

SENSOR_BITS = [
    ("3D gyro", 0x01), ("3D accel", 0x02), ("3D mag", 0x04),
    ("abs pressure", 0x08), ("diff pressure", 0x10), ("GPS", 0x20),
    ("optical flow", 0x40), ("vision position", 0x80),
    ("laser position", 0x100), ("ext ground truth", 0x200),
    ("angular rate control", 0x400), ("attitude stabilization", 0x800),
    ("yaw position", 0x1000), ("z/altitude control", 0x2000),
    ("x/y position control", 0x4000), ("motor outputs", 0x8000),
    ("RC receiver", 0x10000), ("3D gyro 2", 0x20000),
    ("3D accel 2", 0x40000), ("3D mag 2", 0x80000),
    ("geofence", 0x100000), ("AHRS", 0x200000), ("terrain", 0x400000),
    ("reverse motor", 0x800000), ("logging", 0x1000000),
    ("battery", 0x2000000), ("proximity", 0x4000000),
    ("satcom", 0x8000000), ("prearm check", 0x10000000),
    ("obstacle avoidance", 0x20000000), ("propulsion", 0x40000000),
]


def sensor_table(sys_status):
    """[(name, present, enabled, healthy)] for everything the board mentions."""
    rows = []
    for name, bit in SENSOR_BITS:
        present = bool(sys_status.onboard_control_sensors_present & bit)
        enabled = bool(sys_status.onboard_control_sensors_enabled & bit)
        healthy = bool(sys_status.onboard_control_sensors_health & bit)
        if present or enabled or healthy:
            rows.append((name, present, enabled, healthy))
    return rows


FRAME_CLASSES = {
    0: "UNDEFINED -- no motors configured",
    1: "quad", 2: "hexa", 3: "octa", 4: "octaquad", 5: "Y6",
    6: "heli", 7: "tri", 8: "single", 9: "coax", 10: "bicopter",
    11: "heli dual", 12: "dodeca hexa", 13: "heli quad", 14: "deca",
    15: "scripting matrix", 16: "6DoF scripting", 17: "dynamic scripting",
}

FRAME_TYPES = {
    0: "plus", 1: "X", 2: "V", 3: "H", 4: "V-tail", 5: "A-tail",
    10: "Y6B", 11: "Y6F", 12: "betaflight X", 13: "DJI X",
    14: "clockwise X", 15: "I", 18: "betaflight X reversed",
}

# SERVOn_FUNCTION values that matter on a multirotor. 33..40 are Motor 1..8.
SERVO_FUNCTIONS = {
    0: "Disabled", 1: "RCPassThru", 51: "RCIN1", 52: "RCIN2",
    33: "Motor1", 34: "Motor2", 35: "Motor3", 36: "Motor4",
    37: "Motor5", 38: "Motor6", 39: "Motor7", 40: "Motor8",
    41: "Motor9", 42: "Motor10", 43: "Motor11", 44: "Motor12",
    129: "GPIO",
}

MOT_PWM_TYPES = {
    0: "normal PWM (490 Hz)", 1: "OneShot", 2: "OneShot125",
    3: "Brushed", 4: "DShot150", 5: "DShot300", 6: "DShot600",
    7: "DShot1200", 8: "PWM range", 9: "PWM angle",
}

GPS_TYPES = {
    0: "none", 1: "auto", 2: "uBlox", 3: "MTK", 4: "MTK19", 5: "NMEA",
    6: "SiRF", 7: "HIL", 8: "SwiftNav", 9: "DroneCAN", 10: "SBF",
    11: "GSOF", 13: "ERB", 14: "MAV", 15: "NOVA", 16: "HemisphereNMEA",
    17: "uBlox-MovingBaseline-Base", 18: "uBlox-MovingBaseline-Rover",
    19: "MSP", 20: "AllyStar", 21: "ExternalAHRS", 22: "DroneCAN-MovingBaseline-Base",
    23: "DroneCAN-MovingBaseline-Rover", 24: "UnicoreNMEA",
}

BATT_MONITORS = {
    0: "disabled", 3: "analog voltage only", 4: "analog voltage and current",
    5: "solo", 6: "Bebop", 7: "SMBus-Maxell", 8: "UAVCAN-BatteryInfo",
    9: "BLHeli ESC", 10: "SumOfFollowing", 11: "FuelFlow", 12: "FuelLevelPWM",
    13: "SMBUS-SUI3", 14: "SMBUS-SUI6", 15: "NeoDesign", 16: "SMBus-Maxell",
    17: "Generator-Elec", 18: "Generator-Fuel", 19: "Rotoye", 20: "MPPT",
    21: "INA2XX", 22: "LTC2946", 23: "Torqeedo", 24: "FuelLevelAnalog",
}

GPS_FIX_TYPES = {
    0: "no GPS", 1: "no fix", 2: "2D fix", 3: "3D fix", 4: "DGPS",
    5: "RTK float", 6: "RTK fixed", 7: "static", 8: "PPP",
}


def battery_verdict(volts, low_volt=None):
    """Interpret a pack voltage. Returns (state, message).

    state is one of "ok", "low", "absent", "unknown".

    The distinction that matters on a bench: once BATT_MONITOR is enabled with
    nothing plugged in, the analog input floats and reads a volt or three --
    3.43 V was the reading here. Compared naively against BATT_LOW_VOLT that
    looks like a critically flat pack, and a tool that says "battery low"
    sends you looking for a charger instead of a connector. A 3S pack at
    3.4 V would be destroyed, not flat, so a reading far below the failsafe
    means NO PACK, not a discharged one.

    Cell count is inferred from BATT_LOW_VOLT, which the profiles set at about
    3.5 V per cell.
    """
    if low_volt:
        cells = max(1, int(round(low_volt / 3.5)))
    else:
        cells = 3
    if volts < 1.0:
        return ("absent", "%.2f V -- no pack, or the sense lead is unplugged" % volts)
    if volts < cells * 2.5:
        return ("absent",
                "%.2f V -- the analog input is floating, so no pack is connected. "
                "A %dS pack this low would be destroyed rather than flat." % (volts, cells))
    if low_volt and volts < low_volt:
        return ("low", "%.2f V is below BATT_LOW_VOLT %.1f" % (volts, low_volt))
    if low_volt and volts < low_volt + 0.6:
        return ("low", "%.2f V, close to the %.1f V failsafe" % (volts, low_volt))
    return ("ok", "%.2f V" % volts)


def copter_mode_name(custom_mode):
    try:
        return mavutil.mode_string_v10(
            type("hb", (), {"type": mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            "autopilot": mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                            "custom_mode": custom_mode,
                            "base_mode": 0})())
    except Exception:
        return str(custom_mode)


def grep_params(params, pattern):
    """Parameter names matching a regex, numerically sorted where possible."""
    rx = re.compile(pattern)
    names = [n for n in params if isinstance(n, str) and rx.match(n)]

    def key(name):
        digits = re.findall(r"\d+", name)
        return (int(digits[0]) if digits else 0, name)

    return sorted(names, key=key)
