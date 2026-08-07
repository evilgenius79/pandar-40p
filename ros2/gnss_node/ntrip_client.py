#!/usr/bin/env python3
"""NTRIP v1 rover client, transport only. No ROS, no serial, no printing.

Shared by ros2/gnss_node/gnss_node.py (the live rig path) and
scripts/gnss/ntrip_rover.py (the standalone bench tool), so the protocol
lives in exactly one place.

The caller owns the serial port and supplies two callbacks:
    get_gga()  -> the newest GGA sentence, or None if there is no fix yet
    on_rtcm(b) -> hand raw RTCM bytes to the receiver

WHY THE CALLER OWNS THE PORT. Two processes reading the same tty split the
byte stream between them -- each read consumes bytes the other never sees.
Running a separate NTRIP process alongside the GNSS node therefore corrupts
both. One owner, callbacks for the rest.

THE TRAFFIC IS BIDIRECTIONAL and that is not optional. Every InCORS
mountpoint advertises nmea=1, and VRS synthesises observations at the
position you report: send no GGA and there is nothing to synthesise, so the
stream never starts or dies within seconds.

Designed to survive a moving rig on a phone hotspot: any network failure
reconnects with backoff, and nothing here can raise into the caller's
thread. Losing corrections must degrade RTK to autonomous, never take down
position publishing.
"""
import base64
import socket
import threading
import time


class NtripClient:
    def __init__(self, cfg, get_gga, on_rtcm, log=None,
                 gga_interval=5.0, max_backoff=60.0):
        self.cfg = cfg
        self.get_gga = get_gga
        self.on_rtcm = on_rtcm
        self.log = log or (lambda level, msg: None)
        self.gga_interval = gga_interval
        self.max_backoff = max_backoff

        self.run = True
        self.connected = False
        self.rtcm_bytes = 0
        self.last_rtcm = None          # monotonic time of the last RTCM byte
        self.types = {}
        self._thread = None

    # ---------------------------------------------------------------- public
    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.run = False

    def age(self):
        """Seconds since the last RTCM byte, or None if none has arrived."""
        return None if self.last_rtcm is None else time.monotonic() - self.last_rtcm

    # --------------------------------------------------------------- internal
    def _loop(self):
        backoff = 2.0
        while self.run:
            try:
                self._session()
                backoff = 2.0                  # a clean session resets it
            except Exception as exc:           # noqa: BLE001
                # Deliberately broad: a hotspot dropping, DNS failing, the
                # caster resetting -- none of it may escape this thread.
                self.log("warn", f"NTRIP: {exc}; retry in {backoff:.0f}s")
            finally:
                self.connected = False
            for _ in range(int(backoff * 10)):
                if not self.run:
                    return
                time.sleep(0.1)
            backoff = min(backoff * 2, self.max_backoff)

    def _session(self):
        cfg = self.cfg
        # The caster will not stream without a position, so do not even
        # connect until the receiver has one.
        waited = 0.0
        while self.run and self.get_gga() is None:
            time.sleep(0.5)
            waited += 0.5
            if waited > 60 and waited % 60 < 0.5:
                self.log("warn", "NTRIP: still waiting for a GGA (no fix?)")
        if not self.run:
            return

        sock = socket.create_connection(
            (cfg["host"], int(cfg["port"])), timeout=20)
        try:
            req = (f"GET /{cfg['mountpoint']} HTTP/1.0\r\n"
                   f"Host: {cfg['host']}:{cfg['port']}\r\n"
                   f"User-Agent: NTRIP pandar40p/1.0\r\n")
            if cfg.get("user"):
                tok = base64.b64encode(
                    f"{cfg['user']}:{cfg.get('password','')}".encode()).decode()
                req += f"Authorization: Basic {tok}\r\n"
            sock.sendall((req + "Connection: close\r\n\r\n").encode())

            sock.settimeout(20)
            head = sock.recv(4096)
            status = head.split(b"\r\n", 1)[0]
            if b"200" not in status:
                # Bad credentials or a wrong mountpoint will never fix
                # themselves by retrying fast, hence the long sleep.
                raise RuntimeError(
                    f"caster refused: {status.decode('ascii','replace')}")

            self.connected = True
            self.log("info", f"NTRIP connected to {cfg['mountpoint']}")

            body = head.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in head else b""
            pending = body
            last_gga = 0.0
            sock.settimeout(1.0)

            while self.run:
                now = time.monotonic()
                if now - last_gga >= self.gga_interval:
                    g = self.get_gga()
                    if g:
                        sock.sendall((g + "\r\n").encode())
                        last_gga = now

                try:
                    chunk = sock.recv(16384)
                    if not chunk:
                        raise RuntimeError("caster closed the connection")
                    pending += chunk
                except socket.timeout:
                    chunk = b""

                if pending:
                    consumed = 0
                    for mtype, flen in rtcm_frames(pending):
                        self.types[mtype] = self.types.get(mtype, 0) + 1
                        consumed += flen
                    if consumed:
                        self.on_rtcm(pending[:consumed])
                        self.rtcm_bytes += consumed
                        self.last_rtcm = time.monotonic()
                        pending = pending[consumed:]
                    elif len(pending) > (1 << 20):
                        pending = b""          # desync guard
        finally:
            sock.close()


def rtcm_frames(buf):
    """Walk RTCM3 frames, yielding (message_type, total_frame_length).

    Frame: 0xD3 | 6 bits reserved + 10 bits length | payload | 3 byte CRC.
    The message number is the first 12 bits of the payload.
    """
    i = 0
    while i + 3 <= len(buf):
        if buf[i] != 0xD3:
            i += 1
            continue
        length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        end = i + 3 + length + 3
        if end > len(buf):
            return                             # partial frame; wait for more
        if length >= 2:
            yield (buf[i + 3] << 4) | (buf[i + 4] >> 4), end - i
        i = end


def load_config(path):
    """Parse key=value. Returns None if the file is absent or incomplete.

    Absent is a normal state, not an error: the rig must still record
    position with no credentials and no internet.
    """
    import os
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    cfg = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                cfg[k.strip()] = v.strip()
    if {"host", "port", "mountpoint"} - cfg.keys():
        return None
    return cfg


def sourcetable(host, port, timeout=20):
    """Fetch the caster's sourcetable. Unauthenticated by design.

    curl cannot do this: NTRIP v1 replies with the status line
    "SOURCETABLE 200 OK", which is not valid HTTP, so curl aborts.
    """
    s = socket.create_connection((host, int(port)), timeout=timeout)
    s.sendall(f"GET / HTTP/1.0\r\nHost: {host}:{port}\r\n"
              f"User-Agent: NTRIP pandar40p/1.0\r\n"
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
    finally:
        s.close()
    return buf.decode("utf-8", "replace")
