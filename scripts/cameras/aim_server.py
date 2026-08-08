#!/usr/bin/env python3
"""Live camera view for AIMING, served over HTTP — open it on a phone.

    aim_server.py [--port 8081] [--full-res]

    http://<laptop>:8081        both cameras, phone-friendly page
    http://100.117.193.108:8081 over Tailscale, standing at the rig

Exists for next-step 5: the cameras must be aimed (±30–35° splay, 10–15°
up-pitch) and then panel-bonded, which is permanent. Aiming means watching
the image move while your hands are on the camera — impossible with a
laptop-only preview, trivial with the phone clipped next to the rig.

Each board is dual-lens but only ONE lens per board is used (the boards
were the cheap way to buy two sensors, not a stereo pair). Every capture
mode is the combined side-by-side frame, so the stream shows each half
labelled L/R with its own crosshair + thirds grid. Decide per board which
half is the keeper; the other is cropped away in software downstream.

Aiming runs 2560x720@30 by default — lighter on USB 2.0 and CPU, and
alignment does not need full resolution. --full-res switches to
3200x1200@15, the recording configuration (both cameras together CANNOT
do 3200x1200@30; isochronous reservation fails, measured 2026-08-01).

Read-only with respect to the rig: it opens the cameras and nothing else.
Do not run it while recording a bag — it would contend for the same USB
2.0 bandwidth the recording needs.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

# Keeper lens per board -- MATT'S CALL 2026-08-07: A keeps LEFT, B keeps
# RIGHT. The outermost pair, which maximises combined coverage under splay.
# This choice drives the software crop everywhere downstream; change it here
# and in CLAUDE.md together or aiming and mapping will disagree.
# Physical arrangement confirmed by Matt 2026-08-07: board A (video2) is
# mounted on the LEFT side of the mast, board B (video4) on the RIGHT.
# If a wave test ever shows these panels swapped, the USB mapping moved --
# fix it HERE, not by re-mounting.
CAMS = [("/dev/video2", "A  keeper L  (left mount)", "L"),
        ("/dev/video4", "B  keeper R  (right mount)", "R")]

PAGE = """<!doctype html><html><head>
<meta name=viewport content="width=device-width, initial-scale=1">
<title>rig cameras</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:14px system-ui}
 .row{display:flex}
 .row img{width:50%;display:block}
 .note{margin:8px;color:#888}
</style></head><body>
<div class=row><img src="/cam0.mjpg"><img src="/cam1.mjpg"></div>
<p class=note>Keeper lenses only, arranged as mounted: A (points left) on
the left, B (points right) on the right. Green crosshair = optical axis,
goes on the side mark. Amber line = where the CENTRE mark should appear in
each image (~1/3 from the inner edge). When both crosshairs are on their
side marks AND the centre mark sits near both amber lines, splay and
overlap are correct. Up-pitch: horizon on the lower gridline.</p>
</body></html>"""


class Grabber(threading.Thread):
    """Continuously captures one camera; keeps only the newest JPEG."""

    def __init__(self, dev, label, keeper, w, h, fps):
        super().__init__(daemon=True)
        self.dev, self.label, self.keeper = dev, label, keeper
        self.w, self.h, self.fps = w, h, fps
        self.jpeg = None
        self.lock = threading.Lock()
        self.ok = False

    def run(self):
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while True:
            got, frame = cap.read()
            if not got:
                self.ok = False
                time.sleep(0.5)
                continue
            self.ok = True
            # keeper half only -- the discard half is dead weight for aiming
            h, w = frame.shape[:2]
            half = w // 2
            frame = frame[:, :half] if self.keeper == "L" else frame[:, half:]
            frame = frame.copy()
            self.annotate(frame)
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def annotate(self, f):
        """Overlay for a single keeper half.

        The amber vertical line marks where the CENTRE mark should land:
        one third in from the INNER edge -- the edge facing the other
        camera. Camera A points left of forward, so forward appears near
        its RIGHT edge; camera B mirrors that. Crosshair on the side mark
        + centre mark on the amber line = splay correct by construction.
        """
        h, w = f.shape[:2]
        grey, green, amber = (110, 110, 110), (80, 220, 80), (0, 190, 255)
        cx, cy = w // 2, h // 2
        for gx in (w // 3, 2 * w // 3):
            cv2.line(f, (gx, 0), (gx, h), grey, 1)
        for gy in (h // 3, 2 * h // 3):
            cv2.line(f, (0, gy), (w, gy), grey, 1)
        # Centre-mark line: 18% of the width in from the INNER edge (right
        # edge for the left-mounted A, left edge for B). Derivation: the
        # centre mark sits `splay` degrees from each optical axis, so its
        # image position is 0.5 - splay/HFOV from the inner edge; for 33
        # degrees of splay and a ~95-110 degree lens that is 0.15-0.20.
        # An earlier version drew this at 1/3, which is the far EDGE of the
        # overlap zone, not the centre-mark position -- aiming to that line
        # produced half the intended splay. The green-crosshair-on-side-mark
        # method is exact regardless of FOV; this line is the cross-check.
        off = int(w * 0.18)
        fwd_x = w - off if self.keeper == "L" else off
        cv2.line(f, (fwd_x, 0), (fwd_x, h), amber, 2)
        cv2.putText(f, "centre mark", (fwd_x - 150 if self.keeper == "L"
                    else fwd_x + 8, h // 2 - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, amber, 2)
        cv2.line(f, (cx - 40, cy), (cx + 40, cy), green, 3)
        cv2.line(f, (cx, cy - 40), (cx, cy + 40), green, 3)
        cv2.circle(f, (cx, cy), 24, green, 3)
        cv2.putText(f, self.label, (12, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, green, 2)


GRABBERS = []


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        for i, g in enumerate(GRABBERS):
            if self.path == f"/cam{i}.mjpg":
                self.stream(g)
                return
        self.send_response(404)
        self.end_headers()

    def stream(self, g):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with g.lock:
                    jpg = g.jpeg
                if jpg:
                    self.wfile.write(b"--frame\r\n"
                                     b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)          # ~20 fps to the browser, plenty
        except (BrokenPipeError, ConnectionResetError):
            pass                          # phone closed the page

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--full-res", action="store_true",
                    help="3200x1200@15 (recording config) instead of 2560x720@30")
    a = ap.parse_args()

    w, h, fps = (3200, 1200, 15) if a.full_res else (2560, 720, 30)
    for dev, label, keeper in CAMS:
        g = Grabber(dev, label, keeper, w, h, fps)
        g.start()
        GRABBERS.append(g)

    print(f"aim view: http://0.0.0.0:{a.port}   ({w}x{h}@{fps} per camera)")
    print("over Tailscale, on your phone: http://100.117.193.108:%d" % a.port)
    ThreadingHTTPServer(("0.0.0.0", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
