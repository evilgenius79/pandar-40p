#!/usr/bin/env python3
"""Identify a GNSS receiver on a serial port without assuming its baud rate.

Scans candidate rates, scores each by how much of the traffic parses as
NMEA, and reports which talkers and sentence types appear. Talker IDs are
the primary-source answer to "which constellations does this thing
actually track" -- GP/GL/GA/GB/GQ/GI in the stream beats any datasheet.

    gnss_probe.py /dev/ttyACM0
"""
import sys
import time
import serial

import glob, os
# Resolve by USB identity: /dev/ttyACM0 is contested with the XIAO.
_by_id = [q for q in glob.glob("/dev/serial/by-id/*") if "1a86_USB_Single_Serial" in q]
_default = os.path.realpath(_by_id[0]) if len(_by_id) == 1 else "/dev/ttyACM0"
PORT = sys.argv[1] if len(sys.argv) > 1 else _default
RATES = [460800, 115200, 9600, 921600, 230400, 38400, 57600, 19200]

TALKER = {
    "GP": "GPS", "GL": "GLONASS", "GA": "Galileo", "GB": "BeiDou",
    "BD": "BeiDou (legacy)", "GQ": "QZSS", "GI": "NavIC/IRNSS",
    "GN": "combined fix", "PQ": "Quectel proprietary",
}


def nmea_ok(line):
    """Validate an NMEA checksum: XOR of everything between $ and *."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cks = line[1:].partition("*")
    try:
        want = int(cks[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def probe(rate, secs=2.5):
    try:
        s = serial.Serial(PORT, rate, timeout=0.3)
    except serial.SerialException as exc:
        return None, f"{exc}"
    time.sleep(0.2)
    s.reset_input_buffer()
    raw = b""
    end = time.time() + secs
    while time.time() < end:
        raw += s.read(4096)
    s.close()
    lines = raw.decode("ascii", "replace").splitlines()
    good = [l for l in lines if nmea_ok(l.strip())]
    return (raw, good), None


print(f"probing {PORT}\n")
best = None
for rate in RATES:
    res, err = probe(rate)
    if err:
        print(f"  {rate:>7} baud  -- {err}")
        continue
    raw, good = res
    print(f"  {rate:>7} baud  {len(raw):>6} bytes  {len(good):>4} valid NMEA")
    if good and (best is None or len(good) > len(best[1])):
        best = (rate, good, raw)

if not best:
    sys.exit("\nNo valid NMEA at any rate. Wrong port, or the module is in "
             "binary-only mode.")

rate, good, raw = best
print(f"\n=== locked: {rate} baud ===\n")

talkers, types = {}, {}
for line in good:
    tag = line[1:6]
    tk, st = tag[:2], tag[2:5]
    talkers[tk] = talkers.get(tk, 0) + 1
    types[tag] = types.get(tag, 0) + 1

print("talkers seen (this is the constellation evidence):")
for tk, n in sorted(talkers.items(), key=lambda x: -x[1]):
    print(f"  {tk}  {TALKER.get(tk,'?'):<18} {n:>4} sentences")

print("\nsentence types:")
for st, n in sorted(types.items(), key=lambda x: -x[1]):
    print(f"  {st}  {n}")

print("\nsample:")
for line in good[:14]:
    print("  " + line[:150])
