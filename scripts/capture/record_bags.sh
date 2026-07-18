#!/usr/bin/env bash
# Record ALL raw topics for a mapping run. Doctrine: record raw, always —
# every future pipeline improvement re-runs from these bags.
# Usage: ./record_bags.sh [run_name]
set -euo pipefail
RUN="${1:-run_$(date +%Y%m%d_%H%M%S)}"
OUT="${BAG_DIR:-$HOME/mapping_bags}/$RUN"
mkdir -p "$OUT"

echo "[*] Recording to $OUT (Ctrl-C to stop)"
# Topics: adjust to your launch. Defaults below match the planned stack:
#   /lidar_points      - hesai_ros_driver point cloud
#   /lidar_packets     - raw packets if the driver is configured to publish them
#   /imu/data_raw      - imu_bridge_node (200 Hz+)
#   /camera*/image_raw/compressed - colorization frames
#   /fix               - gpsd/nmea GNSS fix
exec ros2 bag record -o "$OUT" \
  /lidar_points /lidar_packets \
  /imu/data_raw \
  /camera0/image_raw/compressed /camera1/image_raw/compressed \
  /fix /tf /tf_static
