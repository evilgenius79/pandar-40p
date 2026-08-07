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

CAMS = [("/dev/video2", "camera A (video2)"),
        ("/dev/video4", "camera B (video4)")]

PAGE = """<!doctype html><html><head>
<meta name=viewport content="width=device-width, initial-scale=1">
<title>rig cameras</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:14px system-ui}
 h2{margin:8px;font-weight:600}
 img{width:100%;display:block}
 .note{margin:8px;color:#888}
</style></head><body>
<h2>camera A (video2)</h2><img src="/cam0.mjpg">
<h2>camera B (video4)</h2><img src="/cam1.mjpg">
<p class=note>Each image is the side-by-side dual-lens frame; only one
half per board will be used. Crosshair marks each lens centre; grid is
thirds. Splay target &plusmn;30&ndash;35&deg;, up-pitch 10&ndash;15&deg;.</p>
</body></html>"""


class Grabber(threading.Thread):
    """Continuously captures one camera; keeps only the newest JPEG."""

    def __init__(self, dev, label, w, h, fps):
        super().__init__(daemon=True)
        self.dev, self.label = dev, label
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
            self.annotate(frame)
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def annotate(self, f):
        h, w = f.shape[:2]
        half = w // 2
        white, grey = (255, 255, 255), (110, 110, 110)
        # separator between the two lenses of this board
        cv2.line(f, (half, 0), (half, h), (0, 200, 255), 2)
        for i, x0 in enumerate((0, half)):
            cx, cy = x0 + half // 2, h // 2
            # thirds grid, then a crosshair at the lens centre
            for gx in (x0 + half // 3, x0 + 2 * half // 3):
                cv2.line(f, (gx, 0), (gx, h), grey, 1)
            for gy in (h // 3, 2 * h // 3):
                cv2.line(f, (x0, gy), (x0 + half, gy), grey, 1)
            cv2.line(f, (cx - 40, cy), (cx + 40, cy), white, 2)
            cv2.line(f, (cx, cy - 40), (cx, cy + 40), white, 2)
            cv2.circle(f, (cx, cy), 24, white, 2)
            cv2.putText(f, ("L", "R")[i], (x0 + 12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, white, 2)
        cv2.putText(f, self.label, (12, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)


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
    for dev, label in CAMS:
        g = Grabber(dev, label, w, h, fps)
        g.start()
        GRABBERS.append(g)

    print(f"aim view: http://0.0.0.0:{a.port}   ({w}x{h}@{fps} per camera)")
    print("over Tailscale, on your phone: http://100.117.193.108:%d" % a.port)
    ThreadingHTTPServer(("0.0.0.0", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
