#!/usr/bin/env bash
# Configure the wired NIC for the Pandar40P (static 192.168.1.100/24, no gateway).
# Usage: sudo ./setup_lidar_nic.sh [iface]   (default: first non-loopback wired iface)
set -euo pipefail

IFACE="${1:-$(ls /sys/class/net | grep -E '^(en|eth)' | head -n1)}"
[ -n "$IFACE" ] || { echo "No wired interface found; pass one explicitly."; exit 1; }

echo "[*] Configuring $IFACE -> 192.168.1.100/24 (no gateway)"
ip addr flush dev "$IFACE"
ip addr add 192.168.1.100/24 dev "$IFACE"
ip link set "$IFACE" up

echo "[*] Link status:"
ethtool "$IFACE" | grep -E "Speed|Duplex|Link detected" || true
echo
echo "Expected with lidar powered: 100Mb/s, Full, Link detected: yes"
echo "Next: scripts/capture/find_lidar.sh $IFACE"
