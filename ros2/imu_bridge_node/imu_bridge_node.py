#!/usr/bin/env python3
"""Serial -> ROS 2 bridge for XIAO firmware v2 (IMU + GPS + PPS).

Demuxes three packet types from one USB-CDC stream:
  0xAA 0x55 (32 B fixed)  -> sensor_msgs/Imu           on /imu/data_raw
  0xAA 0x56 (variable)    -> sensor_msgs/NavSatFix     on /gps/fix   (from GGA)
                             (RMC parsed for validity flag)
  0xAA 0x57 (16 B fixed)  -> sensor_msgs/TimeReference on /gps/pps

Run:  python3 imu_bridge_node.py --port /dev/ttyACM0
Deps: pyserial, rclpy (source ROS 2 Humble first).
"""
import argparse
import struct
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference, Temperature

ACCEL_LSB_PER_G = 4096.0     # must match firmware config (±8 g)
GYRO_LSB_PER_DPS = 32.8      # must match firmware config (±1000 dps)
G = 9.80665
DEG = 3.141592653589793 / 180.0


def crc16_ccitt(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def nmea_deg(field: str, hemi: str) -> float:
    """ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    if not field:
        raise ValueError("empty")
    dot = field.index('.')
    deg = float(field[:dot - 2])
    minutes = float(field[dot - 2:])
    val = deg + minutes / 60.0
    return -val if hemi in ('S', 'W') else val


class ClockMapper:
    """MCU microseconds -> ROS nanoseconds via minimum-latency offset ratchet."""

    def __init__(self):
        self.offset_ns = None

    def map(self, t_us: int, now_ns: int) -> int:
        t_ns = t_us * 1000
        off = now_ns - t_ns
        if self.offset_ns is None or off < self.offset_ns:
            self.offset_ns = off
        return t_ns + int(self.offset_ns)


class SensorBridge(Node):
    def __init__(self, port, baud):
        super().__init__("sensor_bridge")
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", 200)
        self.pub_fix = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.pub_pps = self.create_publisher(TimeReference, "gps/pps", 10)
        # The ICM-42688-P ships a temperature reading in every IMU packet
        # and it was being unpacked and discarded. MEMS bias drifts with
        # temperature, and accel turn-on bias is the known cause of the
        # creeping ax yaw residual, so this is the covariate that makes
        # that correlatable. Decimated to ~1 Hz: 200 Hz of thermal data
        # says nothing extra and would bloat every bag.
        self.pub_temp = self.create_publisher(Temperature, "imu/temperature", 10)
        self.temp_div = 0
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.clock = ClockMapper()
        self.buf = b""
        self.rmc_valid = False
        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"reading {port}")

    # ---------- stream demux ----------
    def poll(self):
        self.buf += self.ser.read(4096)
        while True:
            i = self.buf.find(b"\xaa")
            if i < 0:
                self.buf = b""
                return
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 3:
                return
            ptype = self.buf[1]
            if ptype == 0x55:
                if len(self.buf) < 32:
                    return
                pkt, self.buf = self.buf[:32], self.buf[32:]
                self.handle_imu(pkt)
            elif ptype == 0x57:
                if len(self.buf) < 16:
                    return
                pkt, self.buf = self.buf[:16], self.buf[16:]
                self.handle_pps(pkt)
            elif ptype == 0x56:
                ln = self.buf[2]
                total = 3 + ln + 2
                if ln > 95:                      # implausible: resync
                    self.buf = self.buf[2:]
                    continue
                if len(self.buf) < total:
                    return
                pkt, self.buf = self.buf[:total], self.buf[total:]
                self.handle_gps(pkt, ln)
            else:
                self.buf = self.buf[2:]          # not a packet start: resync

    # ---------- handlers ----------
    def handle_imu(self, pkt):
        if crc16_ccitt(pkt[2:26]) != struct.unpack_from("<H", pkt, 26)[0]:
            return
        t_us, ax, ay, az, gx, gy, gz, temp, seq = struct.unpack_from("<Q6hhH", pkt, 2)
        stamp_ns = self.clock.map(t_us, self.get_clock().now().nanoseconds)
        m = Imu()
        m.header.stamp.sec = stamp_ns // 1_000_000_000
        m.header.stamp.nanosec = stamp_ns % 1_000_000_000
        m.header.frame_id = "imu_link"
        m.linear_acceleration.x = ax / ACCEL_LSB_PER_G * G
        m.linear_acceleration.y = ay / ACCEL_LSB_PER_G * G
        m.linear_acceleration.z = az / ACCEL_LSB_PER_G * G
        m.angular_velocity.x = gx / GYRO_LSB_PER_DPS * DEG
        m.angular_velocity.y = gy / GYRO_LSB_PER_DPS * DEG
        m.angular_velocity.z = gz / GYRO_LSB_PER_DPS * DEG
        m.orientation_covariance[0] = -1.0
        self.pub_imu.publish(m)

        self.temp_div += 1
        if self.temp_div >= 200:                 # ~1 Hz at 200 Hz ODR
            self.temp_div = 0
            t = Temperature()
            t.header.stamp = m.header.stamp
            t.header.frame_id = "imu_link"
            # ICM-42688-P: degC = TEMP_DATA/132.48 + 25. TRANSCRIBED FROM THE
            # DATASHEET, not yet confirmed against a copy in hand -- same
            # caveat as the accel/gyro scale factors.
            t.temperature = temp / 132.48 + 25.0
            t.variance = 0.0
            self.pub_temp.publish(t)

    def handle_pps(self, pkt):
        if crc16_ccitt(pkt[2:12]) != struct.unpack_from("<H", pkt, 12)[0]:
            return
        t_us, seq = struct.unpack_from("<QH", pkt, 2)
        stamp_ns = self.clock.map(t_us, self.get_clock().now().nanoseconds)
        m = TimeReference()
        m.header.stamp.sec = stamp_ns // 1_000_000_000
        m.header.stamp.nanosec = stamp_ns % 1_000_000_000
        m.header.frame_id = "gps_pps"
        m.source = "m10_pps"
        self.pub_pps.publish(m)

    def handle_gps(self, pkt, ln):
        if crc16_ccitt(pkt[2:3 + ln]) != struct.unpack_from("<H", pkt, 3 + ln)[0]:
            return
        try:
            line = pkt[3:3 + ln].decode("ascii", errors="ignore")
        except Exception:
            return
        # strip NMEA checksum suffix if present
        body = line[1:].split('*')[0]
        f = body.split(',')
        talker = f[0]
        if talker.endswith("RMC") and len(f) > 2:
            self.rmc_valid = (f[2] == 'A')
        elif talker.endswith("GGA") and len(f) > 9:
            self.publish_fix(f)

    def publish_fix(self, f):
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "gps"
        try:
            quality = int(f[6]) if f[6] else 0
        except ValueError:
            quality = 0
        if quality == 0:
            m.status.status = NavSatStatus.STATUS_NO_FIX
            self.pub_fix.publish(m)
            return
        try:
            m.latitude = nmea_deg(f[2], f[3])
            m.longitude = nmea_deg(f[4], f[5])
            m.altitude = float(f[9]) if f[9] else 0.0
        except (ValueError, IndexError):
            return
        m.status.status = NavSatStatus.STATUS_FIX
        m.status.service = NavSatStatus.SERVICE_GPS
        # crude covariance from HDOP (field 8): sigma ~ HDOP * 5 m
        try:
            hdop = float(f[8]) if f[8] else 99.0
        except ValueError:
            hdop = 99.0
        var = (hdop * 5.0) ** 2
        m.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, (2*hdop*5.0)**2]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    rclpy.init()
    node = SensorBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
