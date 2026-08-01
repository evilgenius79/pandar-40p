#!/usr/bin/env python3
"""Average stationary accel from a bag's /imu/data_raw to identify the IMU mount.

Reads the .db3 directly with sqlite3 (no rosbag2 metadata, no ros2 CLI) and
deserializes sensor_msgs/msg/Imu with rclpy.

Mount signature (see CLAUDE.md / docs/imu_extrinsic.md). The IMU was
reseated 2026-07-31; the tilt of +Z from gravity is what tells the two
co-mount eras apart, and every bag recorded before that date is on the
crooked one:

  reseated co-mount (2026-07-31 on) -> ay ~ +6.9, az ~ +7.1, tilt ~44.5 deg
  crooked co-mount  (pre 2026-07-31)-> ay ~ +8.5, az ~ +5.1, tilt ~58.9 deg
  flat-mounted      (the original)  -> az ~ +9.8, ay ~ 0,    tilt ~0 deg

The crooked era carries 14.4 deg of undeclared roll error: usable for
SLAM, timestamp and pipeline work, useless for adopting an extrinsic.

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
yaw_resid = math.degrees(math.asin(max(-1.0, min(1.0, amean[0] / mag))))
print(f"  residual yaw from ax    = {yaw_resid:+.1f} deg")
print()

# Classify on tilt angle rather than raw ay/az: it is the quantity the two
# co-mount eras actually differ in, and it is immune to accel scale error.
if tilt < 20.0:
    print("  => FLAT mount signature (tilt ~0 deg) -- pre-co-mount, "
          "large extrinsic error, not usable for SLAM")
elif 38.0 <= tilt <= 51.0:
    print("  => RESEATED CO-MOUNT signature (tilt ~44.5 deg, 2026-07-31 on)")
    print("     geometry believed straight; valid for extrinsic work")
elif 53.0 <= tilt <= 65.0:
    print("  => CROOKED CO-MOUNT signature (tilt ~58.9 deg, pre 2026-07-31)")
    print("     carries 14.4 deg of undeclared roll error about X.")
    print("     OK for SLAM/timestamp/pipeline work; NOT for adopting an")
    print("     extrinsic. See docs/imu_extrinsic.md section 4a.")
else:
    print(f"  => tilt {tilt:.1f} deg matches no known mount era; inspect "
          "manually before trusting this bag")

if abs(yaw_resid) > 3.0:
    print(f"  !! residual yaw {yaw_resid:+.1f} deg is large -- gravity cannot")
    print("     check yaw about the gravity vector, so treat ax as the only")
    print("     handle on it and re-check the board's +Y against the plug")
