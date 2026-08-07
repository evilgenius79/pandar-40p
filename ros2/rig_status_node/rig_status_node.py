#!/usr/bin/env python3
"""Aggregate rig state and serve it as JSON over HTTP.

One place that knows everything: sensor rates, both temperatures, GPS fix,
disk headroom, and whether a bag is recording. Intended to feed a display
(the M5Stack Tab5) over WiFi, but it is equally a phone-friendly status page
over Tailscale, which needs no extra hardware at all.

    rig_status_node.py [--port 8080]
    curl -s localhost:8080 | python3 -m json.tool

Deliberately read-only and dependency-free: http.server from the standard
library, no framework. It publishes nothing to ROS and cannot disturb the
mapping pipeline -- worth caring about on a rig where a QoS mistake once
silently ate two thirds of the lidar data.
"""
import argparse
import json
import os
import shutil
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, Temperature
from std_msgs.msg import UInt8

STATE = {"started": time.time()}
LOCK = threading.Lock()

FIX = {0: "FIX", -1: "NO_FIX", 1: "SBAS", 2: "GBAS"}

# Raw GGA field 6. Only 4 (RTK fixed) is centimetre-class; 5 (float) wanders
# metres on a stationary antenna, measured 2026-08-06.
RTK = {0: "invalid", 1: "autonomous", 2: "DGPS", 3: "PPS",
       4: "RTK FIXED", 5: "RTK float", 6: "dead reckoning"}


class Rate:
    """Message rate over a sliding window, plus time since last message."""

    def __init__(self, window=4.0):
        self.window = window
        self.t = []

    def tick(self):
        now = time.monotonic()
        self.t.append(now)
        self.t = [x for x in self.t if now - x <= self.window]

    def hz(self):
        if len(self.t) < 2:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 0 else 0.0

    def age(self):
        return time.monotonic() - self.t[-1] if self.t else None


class RigStatus(Node):
    def __init__(self):
        super().__init__("rig_status")
        self.r_imu, self.r_lidar, self.r_gps = Rate(), Rate(), Rate()

        # The lidar publishes with BEST_EFFORT on the live path; subscribing
        # RELIABLE there would receive nothing at all. This node only counts
        # messages, so BEST_EFFORT is also the honest choice -- it must not
        # add back-pressure to the pipeline it is observing.
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Imu, "/imu/data_raw", self.on_imu, 50)
        self.create_subscription(PointCloud2, "/lidar_points",
                                 self.on_lidar, sensor_qos)
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gps, 10)
        # NavSatStatus cannot express RTK -- fixed and float both flatten to
        # GBAS, which is the difference between centimetres and metres. The
        # raw GGA quality is what you actually want to see while walking.
        self.create_subscription(UInt8, "/gps/rtk_quality",
                                 self.on_rtk, 10)
        self.create_subscription(Temperature, "/imu/temperature",
                                 lambda m: self.temp("imu", m.temperature), 10)
        self.create_subscription(Temperature, "/lidar/temperature",
                                 lambda m: self.temp("lidar", m.temperature), 10)
        self.create_timer(1.0, self.tick)

    def put(self, k, v):
        with LOCK:
            STATE[k] = v

    def temp(self, which, c):
        """Publish both scales.

        The ROS topics themselves stay in CELSIUS and must -- sensor_msgs/
        Temperature is defined as degrees Celsius, so changing what is
        published would silently break the message contract for anything
        downstream. Fahrenheit belongs in the human-facing layer, which is
        this one.
        """
        with LOCK:
            STATE[f"{which}_temp_f"] = round(c * 9.0 / 5.0 + 32.0, 1)
            STATE[f"{which}_temp_c"] = round(c, 2)

    def on_imu(self, m):
        self.r_imu.tick()
        a = m.linear_acceleration
        g = m.angular_velocity
        amag = (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5
        gmag = (g.x * g.x + g.y * g.y + g.z * g.z) ** 0.5
        with LOCK:
            # Peak accel matters: the accelerometer is +/-8 g and clipping is
            # silent. 42 % of full scale was the worst seen on a sidewalk.
            STATE["accel_mag"] = round(amag, 2)
            STATE["accel_pct_fs"] = round(100 * amag / (8 * 9.80665), 1)
            STATE["gyro_mag"] = round(gmag, 4)
            STATE["still"] = bool(gmag < 0.05)

    def on_lidar(self, m):
        self.r_lidar.tick()
        self.put("lidar_points", m.width * m.height)

    def on_rtk(self, m):
        with LOCK:
            STATE["rtk_quality"] = int(m.data)
            STATE["rtk"] = RTK.get(int(m.data), str(m.data))
            STATE["rtk_ok"] = bool(m.data == 4)

    def on_gps(self, m):
        self.r_gps.tick()
        self.put("gps_status", FIX.get(m.status.status, str(m.status.status)))
        if m.status.status >= 0:
            self.put("gps_lat", round(m.latitude, 7))
            self.put("gps_lon", round(m.longitude, 7))

    def tick(self):
        du = shutil.disk_usage(os.path.expanduser("~"))
        rec = self.recording()
        with LOCK:
            STATE.update({
                "imu_hz": round(self.r_imu.hz(), 1),
                "lidar_hz": round(self.r_lidar.hz(), 2),
                "gps_hz": round(self.r_gps.hz(), 2),
                "imu_age_s": _r(self.r_imu.age()),
                "lidar_age_s": _r(self.r_lidar.age()),
                "disk_free_gb": round(du.free / 1e9, 1),
                # 37.6 MB/s measured = 135 GB/h, so free space converts
                # directly into remaining recording time.
                "record_hours_left": round(du.free / 1e9 / 135.0, 1),
                "recording": rec,
                "uptime_s": int(time.time() - STATE["started"]),
            })

    @staticmethod
    def recording():
        try:
            for pid in os.listdir("/proc"):
                if not pid.isdigit():
                    continue
                try:
                    with open(f"/proc/{pid}/cmdline", "rb") as fh:
                        c = fh.read().decode("utf-8", "replace")
                except OSError:
                    continue
                if "bag" in c and "record" in c:
                    return True
        except OSError:
            pass
        return False


def _r(v):
    return round(v, 2) if v is not None else None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        with LOCK:
            body = json.dumps(STATE, indent=1).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass          # keep the console clean; this is polled continuously


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()

    rclpy.init()
    node = RigStatus()
    srv = HTTPServer(("0.0.0.0", a.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"status JSON on http://0.0.0.0:{a.port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # noqa: BLE001
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    finally:
        srv.shutdown()


if __name__ == "__main__":
    main()
