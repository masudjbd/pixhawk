#!/usr/bin/env python3
"""Live GPS: detection, fix, satellites, HDOP, position wander, with a verdict.

    ./gps_check.py              # 60s, outdoors with a clear sky view
    ./gps_check.py --seconds 20

The distinction this tool exists to make:

  NOT DETECTED means no GPS driver came up at all -- fix_type 0 and no
  satellite count. That is wiring or a dead module, and taking it outside
  will not change it.

  DETECTED, NO FIX means the module is talking and simply cannot see enough
  sky. That is normal indoors and is fixed by going outside, not by changing
  settings.

On this airframe the first state was the one we hit: the GPS module's
magnetometer was being read over I2C while the GPS itself was invisible,
which can only mean the 6-pin serial lead -- unplugged, in the wrong port, or
swapped with the 4-pin one.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

import cubelink as link  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="live GPS check")
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2
    link.request_streams(master, rate_hz=5)

    gps_type = link.get_param(master, "GPS1_TYPE")
    if gps_type is None:
        gps_type = link.get_param(master, "GPS_TYPE")
    serial3 = link.get_param(master, "SERIAL3_PROTOCOL")
    print()
    print("GPS1_TYPE %s (%s), SERIAL3_PROTOCOL %s%s" % (
        gps_type, link.GPS_TYPES.get(int(gps_type or 0), "?"), serial3,
        " (GPS)" if serial3 == 5 else " -- not a GPS port!"))
    print()
    print("sampling %.0fs. Outdoors, clear view of the sky, aircraft still." % args.seconds)
    print()
    print("   time  fix        sats  HDOP  VDOP   lat         lon")

    best_fix, best_sats, best_hdop = 0, 0, 99.99
    positions, samples = [], 0
    start = time.time()
    while time.time() - start < args.seconds:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        samples += 1
        hdop = msg.eph / 100.0
        vdop = msg.epv / 100.0
        best_fix = max(best_fix, msg.fix_type)
        best_sats = max(best_sats, msg.satellites_visible)
        if msg.fix_type >= 3:
            best_hdop = min(best_hdop, hdop)
            positions.append((msg.lat / 1e7, msg.lon / 1e7))
        if samples % 5 == 1:
            print("  %5.0fs  %-10s %4d %5.2f %5.2f   %-11.6f %-11.6f" % (
                time.time() - start,
                link.GPS_FIX_TYPES.get(msg.fix_type, msg.fix_type),
                msg.satellites_visible, hdop, vdop,
                msg.lat / 1e7, msg.lon / 1e7))

    print()
    if samples == 0:
        print("verdict: no GPS_RAW_INT at all -- the board is not streaming.")
        return 1

    print("best seen: %s, %d satellites, HDOP %.2f" % (
        link.GPS_FIX_TYPES.get(best_fix, best_fix), best_sats,
        best_hdop if best_hdop < 99 else float("nan")))

    if best_fix == 0 and best_sats == 0:
        print()
        print("verdict: GPS NOT DETECTED.")
        print("  The driver never came up, so this is not a sky-view problem.")
        print("  Check, in this order:")
        print("    1. the 6-pin GPS lead is in the GPS port, fully seated")
        print("    2. it is not swapped with the 4-pin I2C/compass lead")
        print("    3. the module's own LED lights at all")
        print("    4. SERIAL3_PROTOCOL is 5 (GPS) -- it is %s" % serial3)
        print("    5. if the module is on TELEM1/TELEM2 instead, set that port's")
        print("       protocol to 5, or move the lead")
        print("  If the module's compass shows up in ./wiring_check.py while the")
        print("  GPS does not, the 4-pin lead is fine and the 6-pin one is the")
        print("  fault -- they are separate cables doing separate jobs.")
        return 1

    if best_fix < 3:
        print()
        print("verdict: DETECTED but no 3D fix (%d satellites)." % best_sats)
        print("  The module works. Indoors this is expected -- concrete and")
        print("  glass block it. Take it outside, away from buildings, and")
        print("  allow a cold module two to three minutes for its first fix.")
        return 1

    # Position wander: a static receiver should not drift far.
    if len(positions) > 10:
        lat0 = sum(p[0] for p in positions) / len(positions)
        lon0 = sum(p[1] for p in positions) / len(positions)
        metres = []
        for lat, lon in positions:
            dy = (lat - lat0) * 111320.0
            dx = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
            metres.append(math.sqrt(dx * dx + dy * dy))
        worst = max(metres)
        mean = sum(metres) / len(metres)
        print("position wander while stationary: mean %.1f m, worst %.1f m" % (mean, worst))
        wander_ok = worst < 5.0
    else:
        worst, wander_ok = 0.0, True

    ok = best_sats >= 8 and best_hdop <= 2.0 and wander_ok
    print()
    if ok:
        print("verdict: usable. %d sats, HDOP %.2f, wander %.1f m." % (
            best_sats, best_hdop, worst))
        print("  Position modes (LOITER, RTL, AUTO) have something to work with.")
    else:
        print("verdict: fix present but marginal.")
        if best_sats < 8:
            print("  Under 8 satellites: position modes are not trustworthy.")
        if best_hdop > 2.0:
            print("  HDOP over 2.0: poor satellite geometry. Wait, or move away")
            print("  from buildings and metal.")
        if not wander_ok:
            print("  Wander over 5 m while stationary: LOITER would chase this.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(link.run(main))
