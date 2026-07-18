# Automotive Ethernet (100BASE-T1) Notes

## Why "dead" fleet lidars aren't dead

Fleet-surplus Pandar40Ps (black smooth housing, "Rangefinder" label, Dec-2021 era)
carry a **Broadcom BCM89811** PHY — **100BASE-T1** automotive Ethernet:

- Single twisted pair, full duplex both directions simultaneously (echo cancellation)
- PAM3 signaling — mutually unintelligible with a laptop's 100BASE-TX port
- Symptom on a normal NIC: sensor powers, spins (~0.9–1.5 A), and shows
  **eternal "disconnected" — no link light, ever.** This is expected, not a fault.

Retail Pandar40Ps (silver finned housing) are standard 100BASE-TX and link directly.

## The fix: T1 ↔ TX media converter

Our converter: **BUELEC 100/1000Base-T1-TX-E** (Marvell 88Q2112 + RTL8211FI,
screw-terminal T1 input, MATEnet + H-MTD adapters included, RJ45 out,
6–30 V DC or USB-C power, 100M/1G rate switch, S/M master-slave switch).

Setup:
1. Rate switch → **100M**.
2. Lidar's T1 pair → terminal block (see `pinout.md` for which pair).
3. RJ45 → laptop. Laptop NIC 192.168.1.100/24.
4. Power converter, power lidar.
5. No link? → swap the two T1 wires (polarity), then toggle S/M. One of the four
   combinations links; in practice AUTO/master with correct polarity is typical.
6. Link LED on → proceed exactly as with a normal sensor (find_lidar.sh, web
   control, PandarView, driver).

Alternative converters (verified viable): InnoMaker 100BASE-T1-TX (BCM89811 —
same PHY as the lidar, cables included, ~$60–80); Intrepid RAD-Moon /
RAD-Moon 2 (pro tool; MATEnet/H-MTD cables sold separately — confirm before buying).
Fiber media converters are irrelevant (wrong physical medium, wrong silicon).
1000BASE-T1-only devices do NOT fall back to 100BASE-T1 — the listing must say 100.

## Fleet-config caveats after link-up

A fleet unit may not be at the factory defaults. If link is up but PandarView is
blank:
- `sudo tcpdump -i <iface> -n` and watch for ANY periodic UDP — a live Hesai
  announces itself; its packets reveal the real source IP/ports.
- Then either move the laptop onto that subnet, or reconfigure the sensor
  (web control if reachable, PTC command interface otherwise).
- In PTP clock-source mode the sensor sends no separate GPS UDP packets — normal.

## Junkyard corollary

Any 2017+ vehicle with surround cameras/ADAS is full of T1 links. With a T1
converter on the bench, salvage cameras and ECUs become testable. MATEnet/H-MTD
pigtails can be harvested from late-model camera harnesses.
