#!/usr/bin/env python3
"""Average stationary accel from a bag's /imu/data_raw to identify the IMU mount.

Reads the .db3 directly with sqlite3 (no rosbag2 metadata, no ros2 CLI) and
deserializes sensor_msgs/msg/Imu with rclpy.

Mount signature (see CLAUDE.md / docs/imu_extrinsic.md):
  co-mounted, tilted with the lidar -> ay ~ +8.4, az ~ +5.2
  flat-mounted                      -> az ~ +9.8, ay ~ 0

usage: bag_grav.py <bag_dir_or_db3> [seconds=5.0]

The recording protocol holds the rig dead still for the first 3-5 s, so the
default window is the head of the bag. A high sd means it was not actually
still -- treat the mean as suspect in that case.
"""
import glob
import math
import os
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

path = sys.argv[1]
window = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

if os.path.isdir(path):
    hits = glob.glob(os.path.join(path, "*.db3"))
    if not hits:
        sys.exit(f"no .db3 in {path}")
    db = hits[0]
else:
    db = path

con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
row = con.execute(
    "SELECT id FROM topics WHERE name='/imu/data_raw'").fetchone()
if row is None:
    sys.exit("no /imu/data_raw topic in this bag")
topic_id = row[0]

t0 = con.execute(
    "SELECT MIN(timestamp) FROM messages WHERE topic_id=?", (topic_id,)).fetchone()[0]
cur = con.execute(
    "SELECT timestamp, data FROM messages WHERE topic_id=? AND timestamp<=? "
    "ORDER BY timestamp", (topic_id, t0 + int(window * 1e9)))

ax, ay, az, gx, gy, gz = [], [], [], [], [], []
for _ts, blob in cur:
    m = deserialize_message(bytes(blob), Imu)
    ax.append(m.linear_acceleration.x)
    ay.append(m.linear_acceleration.y)
    az.append(m.linear_acceleration.z)
    gx.append(m.angular_velocity.x)
    gy.append(m.angular_velocity.y)
    gz.append(m.angular_velocity.z)

n = len(ax)
if n == 0:
    sys.exit("no IMU samples in the window")


def ms(v):
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return m, sd


print(f"bag   : {os.path.basename(os.path.dirname(db)) or db}")
print(f"window: first {window:.1f} s, n={n} samples")
for nm, v in (("ax", ax), ("ay", ay), ("az", az)):
    m, sd = ms(v)
    print(f"  {nm} mean={m:+8.4f}  sd={sd:6.4f} m/s^2")
amean = [sum(v) / len(v) for v in (ax, ay, az)]
mag = math.sqrt(sum(c * c for c in amean))
print(f"  |a| = {mag:.4f} m/s^2   (expect ~9.8 if truly stationary)")

gpk = max(math.sqrt(x * x + y * y + z * z) for x, y, z in zip(gx, gy, gz))
print(f"  peak |gyro| = {gpk:.4f} rad/s  (expect <~0.05 if truly still)")

tilt = math.degrees(math.atan2(math.sqrt(amean[0] ** 2 + amean[1] ** 2), amean[2]))
print(f"  tilt of +Z from gravity = {tilt:.1f} deg")
print()
if abs(amean[2]) > 9.0 and abs(amean[1]) < 2.0:
    print("  => FLAT mount signature (az ~ +9.8)")
elif amean[1] > 6.0 and 3.0 < amean[2] < 7.5:
    print("  => CO-MOUNTED / TILTED signature (ay ~ +8.4, az ~ +5.2)")
else:
    print("  => does not match either reference signature; inspect manually")
