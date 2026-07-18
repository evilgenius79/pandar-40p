#!/usr/bin/env bash
# Detect a live Pandar40P and report its actual IP/ports.
# Handles fleet units that may NOT be at the 192.168.1.201 factory default.
# Usage: sudo ./find_lidar.sh [iface]
set -euo pipefail
IFACE="${1:-$(ls /sys/class/net | grep -E '^(en|eth)' | head -n1)}"

command -v tcpdump >/dev/null || { echo "Install tcpdump: sudo apt install tcpdump"; exit 1; }

echo "[*] Listening on $IFACE for lidar traffic (10 s)..."
echo "    Factory default: src 192.168.1.201, point cloud UDP :2368 (broadcast),"
echo "    GPS packets :10110 (absent in PTP clock mode — normal)."
echo

# Capture ANY UDP for 10 s — a live Hesai emits continuously.
timeout 10 tcpdump -i "$IFACE" -n -c 40 udp 2>/dev/null | tee /tmp/lidar_sniff.txt || true

echo
if grep -q "\.2368" /tmp/lidar_sniff.txt; then
  SRC=$(grep "\.2368" /tmp/lidar_sniff.txt | head -1 | awk '{print $3}' | cut -d. -f1-4)
  echo "[+] Point-cloud stream detected. Lidar IP: $SRC"
  echo "    Web control: http://$SRC   PandarView/driver device_ip: $SRC"
elif [ -s /tmp/lidar_sniff.txt ]; then
  echo "[!] UDP traffic present but not on :2368 — fleet config likely."
  echo "    Inspect /tmp/lidar_sniff.txt; the source IP shown is the sensor."
else
  echo "[-] No traffic. Check in order:"
  echo "    1. Lidar powered + spinning (audible, ~0.9-1.5 A @ 12 V)"
  echo "    2. Link up: ethtool $IFACE  (T1 units need the media converter!)"
  echo "    3. T1 converter: rate=100M, try wire-polarity swap, then S/M toggle"
fi
