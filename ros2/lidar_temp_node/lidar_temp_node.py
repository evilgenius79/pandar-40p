#!/usr/bin/env python3
"""Publish the Pandar40P's internal temperature as a ROS 2 topic.

The lidar already measures and reports its own die temperature through the
web console's JSON API -- it costs nothing to record it, and thermal state
is a plausible contributor to any drift that shows up on a hot outdoor run.
The device's own histogram says this unit has spent about 4 hours in the
40-60 C band, and it read 29.6 C cold and 38.8 C after a few hours running
on 2026-08-01.

Publishes sensor_msgs/Temperature on /lidar/temperature. Read-only: it only
ever issues action=get, for the reasons in docs/lidar_console.md.

usage:
    lidar_temp_node.py [--host 192.168.1.201] [--period 5.0]
"""
import argparse
import json
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class LidarTemp(Node):
    def __init__(self, host, period):
        super().__init__("lidar_temp")
        self.url = f"http://{host}/pandar.cgi?action=get&object=TimeStatistic"
        self.pub = self.create_publisher(Temperature, "lidar/temperature", 10)
        self.timer = self.create_timer(period, self.poll)
        self.fails = 0
        self.get_logger().info(f"polling {host} every {period:.1f} s")

    def poll(self):
        try:
            with urllib.request.urlopen(self.url, timeout=3) as r:
                body = json.loads(r.read().decode())["Body"]
            t = float(body["CurrentTemp"])
        except Exception as exc:                      # noqa: BLE001
            # The lidar being unreachable is normal (powered down, unplugged),
            # so warn once rather than filling the log every period.
            self.fails += 1
            if self.fails == 1:
                self.get_logger().warning(f"lidar unreachable: {exc}")
            return
        if self.fails:
            self.get_logger().info("lidar reachable again")
            self.fails = 0

        m = Temperature()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "hesai_lidar"
        m.temperature = t
        m.variance = 0.0
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.201")
    ap.add_argument("--period", type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    node = LidarTemp(a.host, a.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                          # noqa: BLE001
        if type(exc).__name__ != "ExternalShutdownException":
            raise


if __name__ == "__main__":
    main()
