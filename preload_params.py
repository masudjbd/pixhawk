#!/usr/bin/env python3
"""Apply, back up, diff and restore parameter profiles, with every write verified.

    ./preload_params.py --list
    ./preload_params.py --dry-run base frame-quad-x
    ./preload_params.py base frame-quad-x power-module-3s failsafe-3s
    ./preload_params.py --backup
    ./preload_params.py --diff logs/GOLDEN.params
    ./preload_params.py --restore logs/GOLDEN.params

Every write is confirmed by reading back the PARAM_VALUE the board
broadcasts. ArduPilot silently clamps out-of-range writes and ignores
read-only ones, so an unverified write is not a write you can claim
happened.

The distinction --diff is built around, and the reason it is not just a diff:

  CONFIGURATION describes your choices. Nothing physical alters it. If it
  moved, something overwrote it, and pushing the baseline back is right.

  CALIBRATION describes the physical aircraft. When the board shifts on its
  mount, AHRS_TRIM genuinely SHOULD change -- the aircraft really is at a new
  angle, and restoring the old number re-imposes a measurement that is no
  longer true. Re-measure. Never restore.

Confusing the two is how an aircraft gets "fixed" into a worse state, so
--restore touches configuration only unless you force it.
"""

import argparse
import glob
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "lib"))

import cubelink as link  # noqa: E402

PROFILE_DIR = os.path.join(HERE, "profiles")
LOG_DIR = os.path.join(HERE, "logs")

# Parameters that describe the aircraft rather than your intent. Re-measure
# these; never restore them from a file.
CALIBRATION_PATTERNS = [
    r"^AHRS_TRIM_",
    r"^INS_ACC\d*(OFFS|SCAL)_",
    r"^INS_GYR\d*OFFS_",
    r"^INS_(ACC|GYR)\d*_ID$",
    r"^INS_POS\d*_",
    r"^COMPASS_(OFS|DIA|ODI|MOT|SCALE)",
    r"^COMPASS_(DEV_ID|PRIO\d_ID)",
    r"^COMPASS_ORIENT",
    r"^RC\d+_(MIN|MAX|TRIM|DZ)$",
    r"^BARO\d*_(GND_PRESS|DEVID)",
    r"^STAT_",
    r"^SYSID_SW_",
    r"^FORMAT_VERSION$",
    r"^GPS\d*_(POS|DEVID)",
]
CALIBRATION_RX = [re.compile(p) for p in CALIBRATION_PATTERNS]

# Which tool re-measures what, so --diff can say more than "this changed".
REMEASURE_WITH = [
    (re.compile(r"^AHRS_TRIM_"), "./level_calibrate.py"),
    (re.compile(r"^INS_ACC"), "./calibrate_accel.py"),
    (re.compile(r"^COMPASS_(OFS|DIA|ODI|ORIENT)"), "./calibrate_compass.py"),
    (re.compile(r"^RC\d+_(MIN|MAX|TRIM|DZ)$"), "./calibrate_radio.py"),
    (re.compile(r"^INS_GYR\d*OFFS_"), "nothing -- measured at every boot"),
]


def is_calibration(name):
    return any(rx.match(name) for rx in CALIBRATION_RX)


def remeasure_hint(name):
    for rx, tool in REMEASURE_WITH:
        if rx.match(name):
            return tool
    return "re-measure, do not restore"


def profile_path(token):
    """'frame-quad-x' -> profiles/10-frame-quad-x.params."""
    if os.path.exists(token):
        return token
    direct = os.path.join(PROFILE_DIR, token)
    if os.path.exists(direct):
        return direct
    hits = sorted(glob.glob(os.path.join(PROFILE_DIR, "*%s.params" % token)))
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise SystemExit("no profile matching %r (try --list)" % token)
    raise SystemExit("ambiguous profile %r:\n  %s" % (token, "\n  ".join(hits)))


def read_params_file(path):
    """Parse NAME,VALUE or NAME VALUE. Comments and blanks skipped."""
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r"[,\s]+", line, maxsplit=1)
            if len(parts) != 2:
                continue
            try:
                out.append((parts[0].strip().upper(), float(parts[1])))
            except ValueError:
                continue
    return out


def list_profiles():
    print("profiles in %s:" % PROFILE_DIR)
    for path in sorted(glob.glob(os.path.join(PROFILE_DIR, "*.params"))):
        name = os.path.basename(path)
        with open(path) as fh:
            first = ""
            for line in fh:
                if line.startswith("#") and line.strip("# \n"):
                    first = line.strip("# \n")
                    break
        count = len(read_params_file(path))
        print("  %-34s %2d values   %s" % (name, count, first[:60]))
    print()
    print("Apply by any unique suffix: 'base', 'frame-quad-x', 'failsafe-3s'.")


def do_backup(master, label=""):
    os.makedirs(LOG_DIR, exist_ok=True)
    banner = link.parse_banner(link.request_banner(master))
    params = link.fetch_params(master)
    count = params.pop("_count", None)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = "params-%s%s.params" % (stamp, "-%s" % label if label else "")
    path = os.path.join(LOG_DIR, name)
    with open(path, "w") as fh:
        fh.write("# %s %s (%s) board %s -- %s\n" % (
            banner.get("vehicle", "?"), banner.get("version", "?"),
            banner.get("fw_hash", "?"), banner.get("board_id", "?"), stamp))
        fh.write("# %d parameters%s\n" % (len(params),
                                          " of %s reported" % count if count else ""))
        for key in sorted(params):
            fh.write("%s,%.9g\n" % (key, params[key]))
    print("backed up %d parameters -> %s" % (len(params), path))
    return path


def apply_profiles(master, tokens, dry_run):
    wanted = []
    for token in tokens:
        path = profile_path(token)
        values = read_params_file(path)
        print("profile %s (%d values)" % (os.path.basename(path), len(values)))
        wanted.extend(values)

    # Later profiles win, the way the command line reads.
    merged = {}
    for name, value in wanted:
        merged[name] = value

    current = link.fetch_params(master)
    current.pop("_count", None)

    changes, absent, same = [], [], []
    for name in sorted(merged):
        target = merged[name]
        if name not in current:
            absent.append(name)
        elif abs(current[name] - target) <= 1e-6 * max(1.0, abs(target)):
            same.append(name)
        else:
            changes.append((name, current[name], target))

    print()
    print("  %-24s %12s -> %12s" % ("parameter", "current", "proposed"))
    for name, now, target in changes:
        print("  %-24s %12g -> %12g%s" % (
            name, now, target, "   (calibration!)" if is_calibration(name) else ""))
    if same:
        print("  %d already at the requested value" % len(same))
    if absent:
        print("  %d not present in this firmware: %s" % (len(absent), ", ".join(absent)))

    if absent:
        print()
        print("  Some of those may be hidden rather than missing: ArduPilot only")
        print("  publishes a driver's sub-parameters once its parent is enabled, so")
        print("  BATT_VOLT_PIN and friends do not exist until BATT_MONITOR is set.")
        print("  A real apply re-reads the table afterwards and writes them on a")
        print("  second pass.")

    if dry_run:
        print()
        print("dry run -- nothing written.")
        return 0
    if not changes:
        print()
        print("nothing to do.")
        return 0

    print()
    failed = []
    for name, now, target in changes:
        ok, readback = link.set_param(master, name, target)
        status = "ok" if ok else "FAILED"
        print("  write %-24s %g -> %g  %s%s" % (
            name, now, target, status,
            "" if readback is None else " (read back %g)" % readback))
        if not ok:
            failed.append(name)
    # Second pass. Enabling a driver (BATT_MONITOR, GPS_TYPE, a serial
    # protocol) makes its sub-parameters exist, and they were invisible when
    # the table was read. Re-read and write whatever has appeared, rather
    # than telling the user to run the same command twice.
    if absent:
        print()
        print("re-reading the parameter table for values that have appeared")
        again = link.fetch_params(master)
        again.pop("_count", None)
        second = [(name, again[name], merged[name]) for name in absent
                  if name in again
                  and abs(again[name] - merged[name]) > 1e-6 * max(1.0, abs(merged[name]))]
        still_absent = [n for n in absent if n not in again]
        for name, now, value in second:
            ok, readback = link.set_param(master, name, value)
            print("  write %-24s %g -> %g  %s" % (name, now, value,
                                                  "ok" if ok else "FAILED"))
            if not ok:
                failed.append(name)
        if second:
            changes.extend(second)
        if still_absent:
            print("  genuinely absent from this firmware (%d): %s" % (
                len(still_absent), ", ".join(still_absent)))

    print()
    if failed:
        print("%d write(s) did not verify: %s" % (len(failed), ", ".join(failed)))
        print("ArduPilot clamps out-of-range values and ignores read-only ones, so")
        print("an unverified write is not a write. Check the parameter's range.")
        return 1
    print("all %d writes verified." % len(changes))
    return 0


def do_diff(master, path, restore=False, restore_all=False):
    baseline = dict(read_params_file(path))
    current = link.fetch_params(master)
    current.pop("_count", None)

    config_moved, calib_moved, missing = [], [], []
    for name in sorted(baseline):
        if name not in current:
            missing.append(name)
            continue
        if abs(current[name] - baseline[name]) <= 1e-6 * max(1.0, abs(baseline[name])):
            continue
        (calib_moved if is_calibration(name) else config_moved).append(
            (name, current[name], baseline[name]))
    new = sorted(set(current) - set(baseline))

    print("baseline %s" % path)
    print()
    if config_moved:
        print("CONFIGURATION that moved -- restore these:")
        for name, now, was in config_moved:
            print("  %-24s now %12g   baseline %12g" % (name, now, was))
    else:
        print("CONFIGURATION: nothing moved.")
    print()
    if calib_moved:
        print("CALIBRATION that moved -- RE-MEASURE, do not restore:")
        for name, now, was in calib_moved:
            print("  %-24s now %12g   baseline %12g   -> %s" % (
                name, now, was, remeasure_hint(name)))
    else:
        print("CALIBRATION: nothing moved.")
    if missing:
        print()
        print("in the baseline but not in this firmware (%d): %s" % (
            len(missing), ", ".join(missing[:12]) + (" ..." if len(missing) > 12 else "")))
    if new:
        print()
        print("new in this firmware, not in the baseline (%d): %s" % (
            len(new), ", ".join(new[:12]) + (" ..." if len(new) > 12 else "")))

    if not restore:
        return 0

    todo = config_moved + (calib_moved if restore_all else [])
    if calib_moved and not restore_all:
        print()
        print("not restoring the %d calibration value(s): they describe the aircraft,"
              % len(calib_moved))
        print("not your choices. Re-measure them with the tools listed above.")
        print("(--restore --all overrides this, and is almost always wrong.)")
    if not todo:
        print()
        print("nothing to restore.")
        return 0
    print()
    failed = []
    for name, now, was in todo:
        ok, readback = link.set_param(master, name, was)
        print("  restore %-24s %g -> %g  %s" % (name, now, was, "ok" if ok else "FAILED"))
        if not ok:
            failed.append(name)
    return 1 if failed else 0


def do_factory_reset(master):
    """Zero FORMAT_VERSION so ArduPilot rewrites its defaults on next boot."""
    print("This erases ALL parameters: calibration, radio endpoints, failsafes,")
    print("flight modes. The board rewrites firmware defaults on the next boot.")
    print("Back up first (--backup). Type 'reset' to continue: ", end="", flush=True)
    if sys.stdin.readline().strip() != "reset":
        print("aborted.")
        return 1
    ok, readback = link.set_param(master, "FORMAT_VERSION", 0)
    print("FORMAT_VERSION -> 0: %s (read back %s)" % ("ok" if ok else "FAILED", readback))
    if ok:
        print("Power-cycle the board now. Then ./identify.py.")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="parameter profiles, verified")
    ap.add_argument("profiles", nargs="*", help="profile names to apply")
    ap.add_argument("--port")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--backup", action="store_true")
    ap.add_argument("--label", default="", help="suffix for the backup filename")
    ap.add_argument("--diff", metavar="FILE")
    ap.add_argument("--restore", metavar="FILE")
    ap.add_argument("--all", action="store_true",
                    help="with --restore: also restore calibration (almost always wrong)")
    ap.add_argument("--factory-reset", action="store_true")
    ap.add_argument("--reboot", action="store_true",
                    help="reboot the board first, so parameters that a newly "
                         "enabled driver creates are visible")
    args = ap.parse_args()

    if args.list:
        list_profiles()
        return 0
    if not (args.profiles or args.backup or args.diff or args.restore
            or args.factory_reset):
        ap.print_help()
        return 2

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.reboot:
        print("rebooting the board so any newly created parameters appear")
        master = link.reboot(master)
        print("back:  system %d" % master.target_system)

    if args.backup:
        do_backup(master, args.label)
    if args.factory_reset:
        return do_factory_reset(master)
    if args.diff:
        return do_diff(master, args.diff)
    if args.restore:
        return do_diff(master, args.restore, restore=True, restore_all=args.all)
    if args.profiles:
        return apply_profiles(master, args.profiles, args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
