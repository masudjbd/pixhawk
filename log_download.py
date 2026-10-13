#!/usr/bin/env python3
"""List and download dataflash logs over MAVLink.

    ./log_download.py --list
    ./log_download.py --id 7
    ./log_download.py --latest

Logs land in logs/ as .bin, which MAVExplorer, Mission Planner and
plot.ardupilot.org all read.

This board logs to an SD card rather than to a few hundred KB of onboard
dataflash, so logs are large: a few minutes of flight at LOG_BITMASK 196607
is tens of megabytes, and the MAVLink log protocol moves it 90 bytes at a
time. Over USB that is a couple of minutes per 10 MB. Pulling the card and
copying the file is faster whenever the card is reachable -- this tool exists
for when it is not.
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "lib"))

import cubelink as link  # noqa: E402

LOG_DIR = os.path.join(HERE, "logs")
CHUNK = 90


def list_logs(master, timeout=20):
    sys_id, comp = link.target(master)
    master.mav.log_request_list_send(sys_id, comp, 0, 0xFFFF)
    entries = {}
    expected = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.num_logs == 0:
            return {}
        expected = msg.num_logs
        entries[msg.id] = msg
        if len(entries) >= expected:
            break
    return entries


def download(master, log_id, size, path):
    sys_id, comp = link.target(master)
    data = bytearray(size)
    have = set()
    started = time.time()
    offset = 0
    while offset < size:
        master.mav.log_request_data_send(sys_id, comp, log_id, offset, size - offset)
        quiet_since = time.time()
        while time.time() - quiet_since < 2.0:
            msg = master.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.id != log_id:
                continue
            chunk = bytes(bytearray(msg.data)[:msg.count])
            data[msg.ofs:msg.ofs + len(chunk)] = chunk
            have.add(msg.ofs)
            quiet_since = time.time()
            done = len(have) * CHUNK
            if done % (CHUNK * 100) == 0:
                rate = done / max(0.1, time.time() - started) / 1024.0
                print("\r  %6.1f%%  %.0f KB/s" % (100.0 * done / size, rate),
                      end="", flush=True)
        # Resume from the first gap rather than restarting.
        offset = size
        for candidate in range(0, size, CHUNK):
            if candidate not in have:
                offset = candidate
                break
        if offset >= size:
            break
    print()
    with open(path, "wb") as fh:
        fh.write(data)
    return path


def main():
    ap = argparse.ArgumentParser(description="dataflash log download")
    ap.add_argument("--port")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--id", type=int)
    ap.add_argument("--latest", action="store_true")
    args = ap.parse_args()

    try:
        master = link.connect(args.port)
    except link.LinkError as exc:
        print(exc, file=sys.stderr)
        return 2

    entries = list_logs(master)
    if not entries:
        print("the board reports no logs.")
        print("  LOG_BACKEND_TYPE %s" % link.get_param(master, "LOG_BACKEND_TYPE"))
        print("  No SD card, an unmountable card, or logging disabled. A board")
        print("  with no card still flies and still refuses to produce a log.")
        return 1

    print("%d log(s):" % len(entries))
    for log_id in sorted(entries):
        entry = entries[log_id]
        when = time.strftime("%Y-%m-%d %H:%M", time.localtime(entry.time_utc)) \
            if entry.time_utc else "no timestamp"
        print("  %4d  %8.2f MB  %s" % (log_id, entry.size / 1048576.0, when))

    if args.list:
        return 0
    log_id = args.id if args.id is not None else (max(entries) if args.latest else None)
    if log_id is None:
        print()
        print("pass --id N or --latest to download one.")
        return 0
    if log_id not in entries:
        print("no log with id %d" % log_id, file=sys.stderr)
        return 1

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "log-%03d.bin" % log_id)
    size = entries[log_id].size
    print()
    print("downloading log %d (%.2f MB) -> %s" % (log_id, size / 1048576.0, path))
    print("  the MAVLink log protocol moves 90 bytes per message; this is slow")
    download(master, log_id, size, path)
    print("saved %s (%d bytes)" % (path, os.path.getsize(path)))
    return 0


if __name__ == "__main__":
    sys.exit(link.run(main))
