# Pandar40P Bench-Test & Connector-Swap Checklist
*All settings verified against Hesai Pandar40P User Manual (doc 402-en) and PandarView 2 manual. Warranty window: 30 days from Jun 27, 2026 — complete Phase 1–3 ASAP.*

---

## Phase 0 — Prep (before power supply arrives)

- [ ] Download **PandarView 2** from hesaitech.com/en/download (Windows or Ubuntu — it can run on your Windows side for the bench test; SLAM comes later on Ubuntu).
- [ ] Download the **Pandar40P User Manual PDF** from Hesai's site; keep it on the laptop.
- [ ] **Do NOT remove the white protective film from the lens yet** — leave it on until just before first spin-up (protects the lens during handling), but remember it MUST come off before judging point cloud quality.
- [ ] Gather: test hooks / mini grabber clips (6 minimum), multimeter, bench PSU (9–48 V DC, ≥25 W; set 12.0 V, current limit ~2.5 A), inline fuse ~3 A, Ethernet cable, the new 16-pin connector pair, solder + adhesive heat shrink, labels.
- [ ] Print or open the pinout table (below).

### Verified Lemo plug pinout (FGG.2T.316, from manual §2.2.1)

| Pin | Signal | Wire color |
|---|---|---|
| 1–4 | not used | — |
| 5 | Ethernet RX− | Blue |
| 6 | Ethernet RX+ | Blue/White |
| 7 | Ethernet TX− | Orange |
| 8 | Ethernet TX+ | Orange/White |
| 9 | GPS NMEA (RS232 level!) | White |
| 10 | GPS PPS | Yellow |
| 11 | Power V+ | Red |
| 12 | Power V+ | Gray |
| 13 | Ground | Black |
| 14 | Ground | Gray/White |
| 15–16 | not used | Purple, Purple/White |

> Pin numbering reference: manual Figure 10 (view looking into the plug face). Power and ground are **doubled pairs** — always connect both in parallel.

---

## Phase 1 — Pre-cut bench test (temporary hookup)

**Goal: prove the sensor works BEFORE cutting anything, inside the warranty window.**

1. [ ] **Continuity pre-check (unpowered):** beep each Lemo pin to confirm which pins are which if visual pin numbering is uncertain. Confirm NO continuity between pin 11/12 group and pin 13/14 group (no power short).
2. [ ] Attach test hooks to Lemo pins: **11 & 12 → PSU +**, **13 & 14 → PSU −**, **5,6,7,8 → Ethernet**. For the Ethernet pins, sacrifice one end of a patch cable: blue pair of the cable to pins 5/6, orange pair to pins 7/8, keeping pairs twisted to within ~2 cm of the hooks. Leave pins 9/10 unconnected.
3. [ ] Insulate every hook from its neighbors (tape/heat shrink). A short between adjacent pins here is the highest-risk moment of the whole procedure — double-check before power.
4. [ ] Plug the Ethernet into the laptop's gigabit port.
5. [ ] **Configure laptop wired NIC (verified from manual §2.4):**
   - IP: `192.168.1.100`
   - Subnet: `255.255.255.0`
   - No gateway needed. Disable WiFi during the test to keep routing simple.
6. [ ] **Strip the protective film from the lens now.**
7. [ ] Apply power. **The lidar has no power switch — it starts immediately** (verified, manual §2.4). Expect motor spin-up sound and a current draw settling around ~1.5 A @ 12 V (≈18 W typical).

### First-light verification

8. [ ] **Confirm packets (fastest check):** Wireshark on the wired NIC, or on Ubuntu: `sudo tcpdump -i <iface> udp port 2368 -c 5`
   - Expect: UDP from source IP **192.168.1.201** (factory default) to destination **255.255.255.255** (broadcast, factory default), destination port **2368**, 1262-byte UDP payloads. *(All values verified from manual §3.1.)*
9. [ ] **Web control:** browse to `http://192.168.1.201` (Chrome/Firefox, VPN off — verified requirement).
   - [ ] Record from Device Info: serial number, software/firmware versions.
   - [ ] Click **Device Log** → save the JSON (baseline + warranty evidence).
   - [ ] Confirm Spin Rate reads **600 rpm** (factory default = 10 Hz frames).
   - [ ] Confirm **Noise Filtering is OFF**, or run
     `python3 scripts/diagnostics/lidar_config.py` (read-only).
     **This is the one that will waste your day.** With `NoiseFiltering=1`
     the unit spins, streams a full-rate 144k-point cloud, and puts *every
     point at the origin* — nothing errors, so it reads as dead hardware
     and a warranty return. Confirmed by reproduction 2026-08-01; recovery
     is one call, no factory reset (mind the firmware's spelling):
     `curl -s "http://192.168.1.201/pandar.cgi?action=set&object=lidar_data&key=noise_filtring&value=0"`
10. [ ] **PandarView 2:** open live view.
    - [ ] Full 360° point cloud appears, no missing wedges.
    - [ ] **Ranges are non-zero.** All-zero ranges → see the Noise Filtering
      note above, *not* a dead sensor and not the Azimuth FOV page.
    - [ ] All 40 channels returning (view a flat wall: should show 40 distinct scan lines; the 40P imports its calibration file automatically — verified, troubleshooting §8).
    - [ ] No persistent flashing/misaligned points (per manual troubleshooting: flashing points with zero packet loss = software issue, not sensor).
    - [ ] Record a 60-second **PCAP** baseline file and save it.
11. [ ] Let it run 15–30 min. The housing runs warm — normal. (The sensor self-protects: it sets a shutdown flag and powers down 60 s after detecting over-temperature — verified.)
12. [ ] Power down. **Decision gate: only proceed to Phase 2 if every box above is checked.** Any failure → stop, this is a warranty return.

---

## Phase 2 — Cut & splice

1. [ ] Photograph the cable and connector from all angles first (documentation).
2. [ ] Cut mid-cable, leaving **≥ 10 cm of wire on the Lemo plug stub** (keeps the stub usable as a future adapter and as the continuity reference).
3. [ ] **Verify the color map on YOUR cable:** beep every Lemo pin to its wire color on the stub. Check against the table above. If ANY color disagrees with the table, the continuity result wins — relabel accordingly and mark the table.
4. [ ] Label every wire on the sensor side before touching the new connector.
5. [ ] Wire the new 16-pin connector pair. Rules:
   - [ ] Both V+ wires (red, gray) and both GND wires (black, gray/white) carried through — pick pin positions **far away from** the Ethernet positions.
   - [ ] Each Ethernet pair on **adjacent pins**; blue pair and orange pair separated from each other where possible.
   - [ ] Keep each pair **twisted up to the connector shell**; untwist ≤ 2 cm.
   - [ ] If the factory cable has a shield/drain wire, carry it on a spare pin; ground it at the laptop/box end only.
   - [ ] GPS wires (white, yellow) carried through on spares (unused for now — PTP is the sync plan) — remember pin 9 is **RS232-level**, never TTL, if ever used.
   - [ ] Solder + adhesive-lined heat shrink (or proper crimps) on every joint. No twist-and-tape.
6. [ ] On the laptop side of the new connector: power leads → fused battery lead (3 A inline); Ethernet pairs → RJ45 plug (blue pair and orange pair each on standard adjacent RJ45 pair positions — auto-MDIX handles TX/RX orientation).
7. [ ] **Unpowered checks before first mate:** beep every path end-to-end through the mated connector pair; confirm V+ to GND is open-circuit; confirm no Ethernet line touches power.

---

## Phase 3 — Post-splice verification (repeat of first light)

1. [ ] Power up. Same current draw as Phase 1 (a significantly different draw = wiring fault, power off immediately).
2. [ ] `tcpdump`/Wireshark: packets from 192.168.1.201:→2368 present.
3. [ ] Link quality: on Ubuntu `ethtool <iface>` → expect **100 Mb/s, Full duplex**; watch for RX errors: `ip -s link show <iface>` (error counters should stay at 0 while streaming).
4. [ ] Web control reachable; PandarView live cloud identical in quality to the Phase 1 baseline PCAP (compare side by side).
5. [ ] **Wiggle test:** gently flex the cable at the splice and at the new connector while watching the live cloud — zero dropouts/flashing allowed.
6. [ ] Run 30+ minutes streaming; confirm zero packet-loss growth.
7. [ ] Record a fresh baseline PCAP labeled "post-splice."

---

## Quick reference (all verified)

| Item | Value |
|---|---|
| Lidar default IP | 192.168.1.201 |
| Laptop wired IP | 192.168.1.100 / 255.255.255.0 |
| Point cloud | UDP port 2368, broadcast (default) |
| GPS data packets | UDP port 10110 |
| Web control | http://192.168.1.201 (Chrome/Firefox, no VPN) |
| Input power | DC 9–48 V, ~18 W typical (per unit label: max 3 A) |
| Spin rate options | 600 rpm (10 Hz) / 1200 rpm (20 Hz) |
| Clock source options | GPS or **PTP** (web control setting; PTP is our plan) |
| Power switch | None — live on power connect |
| Before GPS wiring (if ever) | Power lidar OFF first (manual warning) |
