#!/usr/bin/env python3
"""Publish the LG290P's position as ROS 2 topics.

    gnss_node.py [--port auto] [--baud 460800]

Replaces the u-blox M10 path. The M10 fed /gps/fix through the XIAO's D5
UART; it was removed from the rig on 2026-08-06 because it is L1-only and
could never use the InCORS L1+L2 correction streams. Without this node
/gps/fix has no publisher at all, and rig.launch.py would happily record an
empty topic -- silently, which is this project's recurring failure mode.

Publishes:
    /gps/fix          sensor_msgs/NavSatFix     position + covariance
    /gps/rtk_quality  std_msgs/UInt8            raw GGA field 6

WHY A SEPARATE QUALITY TOPIC. NavSatStatus has no RTK value -- it offers
only NO_FIX/FIX/SBAS/GBAS. RTK fixed and RTK float both map to GBAS, so the
distinction that actually matters here (centimetres vs metres) would be
destroyed. The raw quality is published alongside so a bag preserves it.

    0 invalid   1 autonomous   2 DGPS   4 RTK FIXED   5 RTK float

COVARIANCE IS APPROXIMATED, and says so. It is derived from HDOP/VDOP when
no $GNGST is available, scaled by a per-quality UERE estimate. That is a
rule-of-thumb, not a measurement -- COVARIANCE_TYPE_APPROXIMATED is set
accordingly. If GST is enabled on the receiver its 1-sigma values are used
instead and the type becomes DIAGONAL_KNOWN.
"""
import argparse
import glob
import math
import os
import sys

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8

GNSS_BY_ID_HINT = "1a86_USB_Single_Serial"

# Rough 1-sigma horizontal error per GGA fix quality, metres. Used only to
# scale DOP into a covariance when the receiver is not sending GST. These
# are conventional figures, not measured on this rig.
UERE = {0: 100.0, 1: 2.5, 2: 1.0, 4: 0.02, 5: 0.4, 6: 50.0}

# GGA quality -> NavSatStatus. RTK has no dedicated enum; both RTK modes are
# ground-based augmentation, so both become STATUS_GBAS_FIX. See the note
# above about why /gps/rtk_quality exists.
STATUS = {0: NavSatStatus.STATUS_NO_FIX,
          1: NavSatStatus.STATUS_FIX,
          2: NavSatStatus.STATUS_SBAS_FIX,
          3: NavSatStatus.STATUS_FIX,
          4: NavSatStatus.STATUS_GBAS_FIX,
          5: NavSatStatus.STATUS_GBAS_FIX,
          6: NavSatStatus.STATUS_NO_FIX}


def resolve_port(requested):
    if requested and requested != "auto":
        return requested
    if os.path.exists("/dev/gnss"):
        return "/dev/gnss"
    m = [p for p in glob.glob("/dev/serial/by-id/*") if GNSS_BY_ID_HINT in p]
    if len(m) == 1:
        return os.path.realpath(m[0])
    sys.exit(f"cannot resolve the GNSS port automatically (matches: {m}). "
             "Pass --port.")


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


def dm_to_deg(v, hemi, degdigits):
    """NMEA ddmm.mmmm -> decimal degrees."""
    d = float(v[:degdigits])
    m = float(v[degdigits:])
    val = d + m / 60.0
    return -val if hemi in ("S", "W") else val


class Lg290pNode(Node):
    def __init__(self, port, baud):
        super().__init__("gnss")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.pub_fix = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.pub_q = self.create_publisher(UInt8, "gps/rtk_quality", 10)
        self.gst = None            # newest (lat_sd, lon_sd, alt_sd) if sent
        self.last_q = None
        self.create_timer(0.02, self.poll)
        self.get_logger().info(f"reading {port} @ {baud}")

    def poll(self):
        for _ in range(40):                    # drain without blocking spin
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                return
            if not raw:
                return
            line = raw.decode("ascii", "replace").strip()
            if not nmea_ok(line):
                continue
            if line[3:6] == "GST":
                self.on_gst(line)
            elif line[3:6] == "GGA":
                self.on_gga(line)

    def on_gst(self, line):
        f = line.split(",")
        try:
            self.gst = (float(f[6]), float(f[7]), float(f[8]))
        except (ValueError, IndexError):
            self.gst = None

    def on_gga(self, line):
        f = line.split(",")
        if len(f) < 10 or not f[2]:
            return
        try:
            q = int(f[6])
            lat = dm_to_deg(f[2], f[3], 2)
            lon = dm_to_deg(f[4], f[5], 3)
            alt = float(f[9])
            sep = float(f[11]) if f[11] else 0.0
            hdop = float(f[8]) if f[8] else 99.0
        except (ValueError, IndexError):
            return

        if q != self.last_q:
            name = {0: "invalid", 1: "autonomous", 2: "DGPS", 4: "RTK FIXED",
                    5: "RTK float", 6: "dead reckoning"}.get(q, "?")
            self.get_logger().info(f"fix quality {self.last_q} -> {q} ({name})")
            self.last_q = q

        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "gnss"
        m.status.status = STATUS.get(q, NavSatStatus.STATUS_NO_FIX)
        # The LG290P tracks GPS+GLONASS+Galileo+BeiDou+QZSS+NavIC. NavSatStatus
        # has no NavIC bit, so it is not representable here.
        m.status.service = (NavSatStatus.SERVICE_GPS |
                            NavSatStatus.SERVICE_GLONASS |
                            NavSatStatus.SERVICE_GALILEO |
                            NavSatStatus.SERVICE_COMPASS)
        m.latitude, m.longitude = lat, lon
        # NavSatFix.altitude is defined as height above the WGS84 ellipsoid.
        # GGA field 9 is orthometric (above MSL) and field 11 is the geoid
        # separation, so the ellipsoidal value is their sum. Publishing the
        # MSL number directly would be wrong by ~35 m here.
        m.altitude = alt + sep

        if self.gst:
            la, lo, al = self.gst
            m.position_covariance = [lo * lo, 0.0, 0.0,
                                     0.0, la * la, 0.0,
                                     0.0, 0.0, al * al]
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            s = hdop * UERE.get(q, 5.0)
            v = 1.7 * s                       # vertical is typically worse
            m.position_covariance = [s * s, 0.0, 0.0,
                                     0.0, s * s, 0.0,
                                     0.0, 0.0, v * v]
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub_fix.publish(m)
        self.pub_q.publish(UInt8(data=q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=460800)
    a = ap.parse_args()
    rclpy.init()
    node = Lg290pNode(resolve_port(a.port), a.baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                          # noqa: BLE001
        if type(exc).__name__ != "ExternalShutdownException":
            raise


if __name__ == "__main__":
    main()
