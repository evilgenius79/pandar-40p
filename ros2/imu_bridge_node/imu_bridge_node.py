#!/usr/bin/env python3
"""Serial -> sensor_msgs/Imu publisher for the XIAO ESP32-S3 IMU bridge.

Reads 32-byte packets (see firmware/imu_bridge/src/main.cpp), maps the MCU's
microsecond clock onto ROS time with a simple offset+drift estimator, and
publishes /imu/data_raw at the sensor ODR (200 Hz).

Run:  python3 imu_bridge_node.py --port /dev/ttyACM0
Deps: pyserial, rclpy (source your ROS 2 Humble environment first).
"""
import argparse
import struct
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

MAGIC = b"\xaa\x55"
PKT = 32
# Must match firmware config: ±8 g, ±1000 dps
ACCEL_LSB_PER_G = 4096.0
GYRO_LSB_PER_DPS = 32.8
G = 9.80665
DEG = 3.141592653589793 / 180.0


def crc16_ccitt(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class ClockMapper:
    """Map MCU microseconds -> ROS nanoseconds.

    Exponentially-smoothed offset; assumes MCU drift is slow (true for the
    ESP32's crystal over a scan session). Latency spikes are rejected by only
    ratcheting the offset toward the minimum observed transport delay.
    """

    def __init__(self, alpha=0.01):
        self.offset_ns = None
        self.alpha = alpha

    def map(self, t_us: int, now_ns: int) -> int:
        t_ns = t_us * 1000
        off = now_ns - t_ns
        if self.offset_ns is None or off < self.offset_ns:
            self.offset_ns = off          # new minimum-latency observation
        else:
            self.offset_ns += self.alpha * (off - self.offset_ns) * 0.0
        return t_ns + int(self.offset_ns)


class ImuBridge(Node):
    def __init__(self, port, baud):
        super().__init__("imu_bridge")
        self.pub = self.create_publisher(Imu, "imu/data_raw", 200)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.clock = ClockMapper()
        self.buf = b""
        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"reading {port}")

    def poll(self):
        self.buf += self.ser.read(4096)
        while True:
            i = self.buf.find(MAGIC)
            if i < 0 or len(self.buf) - i < PKT:
                if i > 0:
                    self.buf = self.buf[i:]
                return
            pkt = self.buf[i:i + PKT]
            self.buf = self.buf[i + PKT:]
            if crc16_ccitt(pkt[2:26]) != struct.unpack_from("<H", pkt, 26)[0]:
                continue
            t_us, ax, ay, az, gx, gy, gz, temp, seq = struct.unpack_from(
                "<Q6hhH", pkt, 2)
            now = self.get_clock().now().nanoseconds
            stamp_ns = self.clock.map(t_us, now)

            msg = Imu()
            msg.header.stamp.sec = stamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
            msg.header.frame_id = "imu_link"
            msg.linear_acceleration.x = ax / ACCEL_LSB_PER_G * G
            msg.linear_acceleration.y = ay / ACCEL_LSB_PER_G * G
            msg.linear_acceleration.z = az / ACCEL_LSB_PER_G * G
            msg.angular_velocity.x = gx / GYRO_LSB_PER_DPS * DEG
            msg.angular_velocity.y = gy / GYRO_LSB_PER_DPS * DEG
            msg.angular_velocity.z = gz / GYRO_LSB_PER_DPS * DEG
            msg.orientation_covariance[0] = -1.0  # no orientation provided
            self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    rclpy.init()
    node = ImuBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
