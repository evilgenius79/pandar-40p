#!/usr/bin/env bash
# Run the laptop as PTP (IEEE 1588) master on the lidar link.
# The Pandar40P's Clock Source must be set to PTP in web control (http://192.168.1.201).
# In PTP mode the sensor stamps points with PTP time and sends no GPS UDP packets.
# Usage: sudo ./ptp_master.sh [iface]
set -euo pipefail
IFACE="${1:-$(ls /sys/class/net | grep -E '^(en|eth)' | head -n1)}"

command -v ptp4l >/dev/null || { echo "Install linuxptp: sudo apt install linuxptp"; exit 1; }

echo "[*] Starting ptp4l as master on $IFACE (Ctrl-C to stop)"
# -m: log to stdout   -S: software timestamping (works on any NIC; the lidar only
# needs a consistent clock to follow — hardware timestamping is a bonus if the NIC
# supports it; try without -S first and fall back if ptp4l complains).
exec ptp4l -i "$IFACE" -m -S --masterOnly 1
