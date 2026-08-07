#!/usr/bin/env python3
"""Send Quectel PQTM query commands and print the replies.

READ-ONLY BY CONSTRUCTION. Quectel's convention is that a config command
takes a mode field: ",R" reads, ",W" writes. Every command in QUERIES below
is either a pure query (VERNO) or an explicit ",R". Nothing here changes
receiver state -- keep it that way, and add new commands only in the R form.

    gnss_query.py /dev/ttyACM0 [baud]
"""
import sys
import time
import serial

import glob, os
# Resolve by USB identity: /dev/ttyACM0 is contested with the XIAO.
_by_id = [q for q in glob.glob("/dev/serial/by-id/*") if "1a86_USB_Single_Serial" in q]
_default = os.path.realpath(_by_id[0]) if len(_by_id) == 1 else "/dev/ttyACM0"
PORT = sys.argv[1] if len(sys.argv) > 1 else _default
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

QUERIES = [
    ("PQTMVERNO",           "firmware / model string"),
    ("PQTMCFGCNST,R",       "constellations enabled"),
    ("PQTMCFGFIXRATE,R",    "position fix rate"),
    ("PQTMCFGRCVRMODE,R",   "receiver mode (1=rover 2=base)"),
    ("PQTMCFGUART,R",       "UART settings"),
    ("PQTMCFGPROT,R,1,1",   "protocol in/out on UART1"),
    ("PQTMCFGSAT,R,1,1",    "GPS satellite mask"),
    ("PQTMCFGSVIN,R",       "survey-in (base) config"),
]


def nmea(body):
    """Wrap a command body in $...*CC with an XOR checksum."""
    ck = 0
    for c in body:
        ck ^= ord(c)
    return f"${body}*{ck:02X}\r\n".encode()


s = serial.Serial(PORT, BAUD, timeout=0.4)
time.sleep(0.3)

for body, label in QUERIES:
    name = body.split(",")[0]
    s.reset_input_buffer()
    s.write(nmea(body))
    s.flush()

    replies, end = [], time.time() + 1.6
    while time.time() < end:
        raw = s.readline().decode("ascii", "replace").strip()
        if raw.startswith("$" + name):
            replies.append(raw)
            if len(replies) >= 3:
                break

    print(f"--- {name:<18} {label}")
    if replies:
        for r in replies:
            print(f"    {r}")
    else:
        print("    (no reply -- command unsupported on this firmware)")
    print()

s.close()
