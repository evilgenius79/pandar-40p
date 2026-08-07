#!/usr/bin/env python3
"""Live GNSS/RTK monitor — for aiming an antenna and watching RTK engage.

    gnss_monitor.py                 # with corrections, so RTK can actually fix
    gnss_monitor.py --no-ntrip      # signal survey only
    gnss_monitor.py --once          # one snapshot, then exit

Prints a refreshing summary: fix quality, correction age, and C/N0 broken
down BY BAND. The band split is the point. RTK resolves integers using the
signals the corrections cover, and the second frequency is always weaker
than the first -- so a healthy-looking overall median can still hide an L2
that is too weak to fix with.

InCORS MSM4 carries these pairs (1074/1084/1094/1124):
    GPS      L1 C/A + L2C          GLONASS  G1 + G2
    Galileo  E1 + E5b              BeiDou   B1I + B2I
Signals outside those pairs (L5, E6, B2a, B3I...) are tracked by the
receiver but have no corrections, so they do not help RTK.

Rules of thumb for the second frequency, on a stationary antenna:
    >= 40 dBHz   should fix within a minute or two
    35-40        float, may fix eventually
    < 35         float forever -- move the antenna, do not wait

Signal IDs are from the Quectel LG290P protocol spec v1.0, Table 8.
"""
import argparse
import collections
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "ros2", "gnss_node"))
from ntrip_client import NtripClient, load_config          # noqa: E402

GNSS_BY_ID_HINT = "1a86_USB_Single_Serial"

# (talker, signalId) -> (band label, is it in the InCORS correction set?)
BANDS = {
    ("GP", "1"): ("GPS L1 C/A", True),  ("GP", "6"): ("GPS L2C", True),
    ("GP", "8"): ("GPS L5-Q", False),
    ("GL", "1"): ("GLO G1 C/A", True),  ("GL", "3"): ("GLO G2 C/A", True),
    ("GA", "7"): ("GAL E1", True),      ("GA", "2"): ("GAL E5b", True),
    ("GA", "1"): ("GAL E5a", False),    ("GA", "5"): ("GAL E6", False),
    ("GB", "1"): ("BDS B1I", True),     ("GB", "B"): ("BDS B2I", True),
    ("GB", "3"): ("BDS B1C", False),    ("GB", "5"): ("BDS B2a", False),
    ("GB", "6"): ("BDS B2b", False),    ("GB", "8"): ("BDS B3I", False),
    ("GQ", "1"): ("QZSS L1", False),    ("GQ", "6"): ("QZSS L2C", False),
    ("GQ", "8"): ("QZSS L5", False),    ("GI", "1"): ("NavIC L5", False),
}
FIX = {"0": "invalid", "1": "autonomous", "2": "DGPS", "3": "PPS",
       "4": "RTK FIXED", "5": "RTK float", "6": "dead reckoning"}


def resolve_port(req):
    import glob
    if req and req != "auto":
        return req
    if os.path.exists("/dev/gnss"):
        return "/dev/gnss"
    m = [p for p in glob.glob("/dev/serial/by-id/*") if GNSS_BY_ID_HINT in p]
    if len(m) == 1:
        return os.path.realpath(m[0])
    sys.exit(f"cannot resolve the GNSS port (matches: {m}); pass --port")


def nmea_ok(line):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--config", default="~/.config/ntrip/incors.conf")
    ap.add_argument("--no-ntrip", action="store_true")
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()

    ser = serial.Serial(resolve_port(a.port), a.baud, timeout=0.3)
    state = {"gga": None}
    ntrip = None
    if not a.no_ntrip:
        cfg = load_config(a.config)
        if cfg:
            ntrip = NtripClient(cfg, get_gga=lambda: state["gga"],
                                on_rtcm=ser.write,
                                log=lambda lvl, m: print(f"  [ntrip] {m}"))
            ntrip.start()
        else:
            print(f"  no NTRIP config at {a.config}; signal survey only")

    t_fix = None
    try:
        while True:
            best = {}                      # (talker,sv,sig) -> best C/N0
            end = time.time() + a.interval
            while time.time() < end:
                line = ser.readline().decode("ascii", "replace").strip()
                if not nmea_ok(line):
                    continue
                if line[3:6] == "GGA" and line.split(",")[2]:
                    state["gga"] = line
                elif line[3:6] == "GSV":
                    f = line.split("*")[0].split(",")
                    talker, sig = f[0][1:3], f[-1]
                    i = 4
                    while i + 3 < len(f):
                        sv, cno = f[i], f[i + 3]
                        if sv.strip() and cno.strip():
                            k = (talker, sv, sig)
                            best[k] = max(best.get(k, 0), int(cno))
                        i += 4

            g = (state["gga"] or "").split(",")
            q = g[6] if len(g) > 6 else "?"
            age = g[13] if len(g) > 13 and g[13] else None
            if q == "4" and t_fix is None:
                t_fix = time.time()

            print(f"\n=== {time.strftime('%H:%M:%S')} "
                  f"fix {q} ({FIX.get(q,'?')})   "
                  f"{g[7] if len(g)>7 else '?'} sats   "
                  f"HDOP {g[8] if len(g)>8 else '?'}   "
                  f"corr age {age + ' s' if age else 'NONE'}")

            grp = collections.defaultdict(list)
            for (t, sv, sig), c in best.items():
                label, used = BANDS.get((t, sig), (f"{t} sig{sig}", False))
                grp[(label, used)].append(c)

            print(f"  {'band':<12}{'used':<6}{'n':>3}  {'median':>7}"
                  f"  {'>=40':>5}  {'best':>5}")
            print("  " + "-" * 46)
            for (label, used), v in sorted(grp.items(),
                                           key=lambda x: (not x[0][1], x[0][0])):
                v.sort(reverse=True)
                med = v[len(v) // 2]
                flag = "RTK" if used else "  -"
                warn = ""
                if used and med < 35:
                    warn = "  <-- too weak to fix"
                elif used and med < 40:
                    warn = "  <-- marginal"
                print(f"  {label:<12}{flag:<6}{len(v):>3}  {med:>7}"
                      f"  {sum(1 for x in v if x >= 40):>5}  {v[0]:>5}{warn}")

            if ntrip:
                print(f"  RTCM {ntrip.rtcm_bytes/1024:.1f} kB, "
                      f"{'connected' if ntrip.connected else 'DISCONNECTED'}")
            if a.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if ntrip:
            ntrip.stop()
        ser.close()


if __name__ == "__main__":
    main()
