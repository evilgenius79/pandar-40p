#!/usr/bin/env python3
"""NTRIP rover client: pipe network RTK corrections into the LG290P.

    ntrip_rover.py [--config ~/.config/ntrip/incors.conf] [--port /dev/ttyACM0]
    ntrip_rover.py --sourcetable          # list mountpoints, connect to none

Why this exists rather than `ros-humble-ntrip-client`: the ROS package is
the right long-term home, but this proves the whole chain -- caster auth,
GGA upstream, RTCM downstream, receiver actually entering RTK -- with no
ROS running and nothing to rebuild. Run this first. If the fix quality does
not reach 4 here, no amount of ROS wiring will help.

THE TRAFFIC IS BIDIRECTIONAL, and that is not optional. Every InCORS
mountpoint advertises nmea=1 in the sourcetable, and MSM4_VRS is a *virtual*
reference station: the caster synthesises observations at the position you
report. Send no GGA and there is nothing for it to synthesise, so the stream
either never starts or dies after a few seconds.

CREDENTIALS LIVE OUTSIDE THIS REPO. pandar-40p is public on GitHub. The
config file is ~/.config/ntrip/incors.conf, mode 600. Never inline them here
and never commit a filled-in config.

Fix quality, GGA field 6 -- this is the number that matters:
    0 invalid   1 autonomous   2 DGPS   4 RTK FIXED   5 RTK float
Expect 1 -> 5 within seconds of corrections flowing, then 5 -> 4 once the
integers resolve. 4 is centimetre-class; 5 is decimetre.
"""
import argparse
import base64
import os
import socket
import sys
import threading
import time

import serial

FIX = {"0": "invalid", "1": "autonomous", "2": "DGPS", "3": "PPS",
       "4": "RTK FIXED", "5": "RTK float", "6": "dead reckoning"}

# RTCM3 message types worth naming. The MSM4 set is the point of the
# exercise: 1074/1084/1094/1124 are GPS/GLONASS/Galileo/BeiDou. Seeing all
# four arrive is the proof that MSM4_VRS beats the RTCM3_* mountpoints,
# which carry no Galileo or BeiDou at all.
RTCM = {1005: "station coords", 1006: "station coords+height",
        1008: "antenna descriptor", 1033: "receiver descriptor",
        1074: "MSM4 GPS", 1084: "MSM4 GLONASS",
        1094: "MSM4 Galileo", 1124: "MSM4 BeiDou",
        1075: "MSM5 GPS", 1085: "MSM5 GLONASS",
        1095: "MSM5 Galileo", 1125: "MSM5 BeiDou",
        1077: "MSM7 GPS", 1087: "MSM7 GLONASS",
        1097: "MSM7 Galileo", 1127: "MSM7 BeiDou",
        1230: "GLONASS code-phase biases", 4094: "proprietary"}

state = {"gga": None, "quality": None, "sats": None, "run": True,
         "rtcm_bytes": 0, "types": {}, "first_rtk": None, "t0": time.time()}
lock = threading.Lock()


def load_config(path):
    cfg = {}
    with open(os.path.expanduser(path)) as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                cfg[k.strip()] = v.strip()
    missing = {"host", "port", "mountpoint"} - cfg.keys()
    if missing:
        sys.exit(f"config {path} is missing: {', '.join(sorted(missing))}")
    return cfg


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


def reader(ser):
    """Consume NMEA from the receiver; keep the newest valid GGA."""
    while state["run"]:
        try:
            line = ser.readline().decode("ascii", "replace").strip()
        except (serial.SerialException, OSError) as exc:
            print(f"[serial] read failed: {exc}")
            state["run"] = False
            return
        if not line.startswith("$") or "GGA" not in line[:8]:
            continue
        if not nmea_ok(line):
            continue
        f = line.split(",")
        if len(f) < 8 or not f[2]:
            continue                      # no position yet
        with lock:
            state["gga"] = line
            q, prev = f[6], state["quality"]
            state["quality"], state["sats"] = q, f[7]
            if q != prev:
                el = time.time() - state["t0"]
                print(f"[{el:6.1f}s] fix quality {prev} -> {q}  "
                      f"({FIX.get(q, '?')}), {f[7]} sats")
                if q in ("4", "5") and state["first_rtk"] is None:
                    state["first_rtk"] = el
                    print(f"[{el:6.1f}s] *** RTK acquired after {el:.1f} s ***")


def rtcm_types(buf):
    """Walk RTCM3 frames, yielding (type, total_frame_len).

    Frame: 0xD3 | 6 bits reserved + 10 bits length | payload | 3 byte CRC.
    Message number is the first 12 bits of the payload.
    """
    i = 0
    while i + 3 <= len(buf):
        if buf[i] != 0xD3:
            i += 1
            continue
        length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        end = i + 3 + length + 3
        if end > len(buf):
            return                        # partial frame; wait for more
        if length >= 2:
            yield (buf[i + 3] << 4) | (buf[i + 4] >> 4), end - i
        i = end


def sourcetable(cfg):
    s = socket.create_connection((cfg["host"], int(cfg["port"])), timeout=20)
    s.sendall(f"GET / HTTP/1.0\r\nUser-Agent: NTRIP ntrip_rover/1.0\r\n"
              f"Connection: close\r\n\r\n".encode())
    buf = b""
    try:
        while True:
            c = s.recv(65536)
            if not c:
                break
            buf += c
    except socket.timeout:
        pass
    s.close()
    for line in buf.decode("utf-8", "replace").splitlines():
        if line.startswith("STR;"):
            f = line.split(";")
            nmea = "GGA required" if len(f) > 11 and f[11] == "1" else "-"
            print(f"  {f[1]:<20} {f[3]:<8} {f[6]:<20} {nmea}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="~/.config/ntrip/incors.conf")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--gga-interval", type=float, default=5.0,
                    help="seconds between GGA reports upstream")
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after N seconds (0 = run until Ctrl-C)")
    ap.add_argument("--sourcetable", action="store_true")
    ap.add_argument("--no-inject", action="store_true",
                    help="receive RTCM but do NOT write it to the receiver")
    a = ap.parse_args()

    cfg = load_config(a.config)
    if a.sourcetable:
        sourcetable(cfg)
        return

    ser = serial.Serial(a.port, a.baud, timeout=0.5)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()

    # The caster will not stream until it has a position, so wait for the
    # receiver's own fix before opening the connection.
    print(f"waiting for a GGA from {a.port} @ {a.baud} ...")
    for _ in range(150):
        with lock:
            if state["gga"]:
                break
        time.sleep(0.1)
    with lock:
        if not state["gga"]:
            sys.exit("no GGA in 15 s -- check antenna and sky view")
        print(f"got fix: quality {state['quality']} "
              f"({FIX.get(state['quality'], '?')}), {state['sats']} sats")

    url = f"{cfg['host']}:{cfg['port']}/{cfg['mountpoint']}"
    print(f"connecting to {url}")
    sock = socket.create_connection((cfg["host"], int(cfg["port"])), timeout=20)
    req = (f"GET /{cfg['mountpoint']} HTTP/1.0\r\n"
           f"Host: {cfg['host']}:{cfg['port']}\r\n"
           f"User-Agent: NTRIP ntrip_rover/1.0\r\n")
    if cfg.get("user"):
        tok = base64.b64encode(
            f"{cfg['user']}:{cfg.get('password','')}".encode()).decode()
        req += f"Authorization: Basic {tok}\r\n"
    sock.sendall((req + "Connection: close\r\n\r\n").encode())

    sock.settimeout(15)
    head = sock.recv(4096)
    first = head.split(b"\r\n", 1)[0].decode("ascii", "replace")
    print(f"caster says: {first}")
    if b"200" not in head.split(b"\r\n", 1)[0]:
        sys.exit(f"caster refused the connection:\n{head.decode('utf-8','replace')[:400]}")

    # Anything after the header is already RTCM.
    body = head.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in head else b""
    print(f"streaming{' (RTCM NOT injected: --no-inject)' if a.no_inject else ''}"
          f" -- Ctrl-C to stop\n")

    sock.settimeout(1.0)
    pending, last_gga, last_report = body, 0.0, time.time()
    deadline = time.time() + a.seconds if a.seconds else None
    try:
        while state["run"]:
            now = time.time()
            if deadline and now > deadline:
                break

            # ---- GGA upstream: what makes VRS possible at all
            if now - last_gga >= a.gga_interval:
                with lock:
                    g = state["gga"]
                if g:
                    sock.sendall((g + "\r\n").encode())
                last_gga = now

            # ---- RTCM downstream
            try:
                chunk = sock.recv(16384)
                if not chunk:
                    print("[caster] closed the connection")
                    break
                pending += chunk
            except socket.timeout:
                chunk = b""

            if pending:
                consumed = 0
                for mtype, flen in rtcm_types(pending):
                    with lock:
                        state["types"][mtype] = state["types"].get(mtype, 0) + 1
                    consumed += flen
                if not a.no_inject and consumed:
                    ser.write(pending[:consumed])
                with lock:
                    state["rtcm_bytes"] += consumed
                pending = pending[consumed:] if consumed else pending
                if len(pending) > 1 << 20:
                    pending = b""          # desync guard

            if now - last_report >= 10:
                last_report = now
                with lock:
                    kb = state["rtcm_bytes"] / 1024
                    q = state["quality"]
                    tl = ", ".join(
                        f"{t}({RTCM.get(t, '?')})×{n}"
                        for t, n in sorted(state["types"].items()))
                print(f"[{now - state['t0']:6.1f}s] {kb:8.1f} kB RTCM | "
                      f"fix {q} ({FIX.get(q, '?')})")
                print(f"          {tl}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        state["run"] = False
        sock.close()
        ser.close()
        with lock:
            print(f"\n--- summary ---")
            print(f"  RTCM received   {state['rtcm_bytes']/1024:.1f} kB")
            print(f"  final fix       {state['quality']} "
                  f"({FIX.get(state['quality'], '?')})")
            if state["first_rtk"]:
                print(f"  time to RTK     {state['first_rtk']:.1f} s")
            else:
                print("  time to RTK     never reached 4 or 5")
            for t, n in sorted(state["types"].items()):
                print(f"  {t:<6} {RTCM.get(t, '?'):<28} {n}")


if __name__ == "__main__":
    main()
