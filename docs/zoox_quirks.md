# Zoox Fleet Pandar40P — Quirks, Traps & Fixes

These units ("PA40-Zoox" in web console Device Info, smooth black housing,
"Rangefinder" label, Dec-2021 era, surplus via recyclers) differ from retail
Pandar40Ps in ways that cost us weeks. Everything below is verified on serial
PA4038C35C9738C15F, fw 2.20.17. **Read this before powering one.**

## 1. Ethernet is 100BASE-T1 automotive, not standard TX
- PHY: **Broadcom BCM89811** (confirmed by teardown chip photo).
- Symptom on a normal NIC: powers, spins (~0.9 A lasers-off / more lasers-on),
  **eternal "disconnected", no link light.** The unit is NOT defective.
- Fix: 100BASE-T1↔TX media converter. Confirmed working: **BUELEC
  100/1000Base-T1-TX-E — rate switch 100M, mode MASTER, lidar's ORANGE pair
  (Lemo pins 7/8) into the terminal block.** Blue pair (5/6) is unused on T1
  variants. If no link: swap wire polarity, then toggle S/M.
- Full background: [t1_ethernet.md](t1_ethernet.md).

## 2. THE ZERO-RANGES TRAP (laser_enable / Azimuth FOV)
The worst one. Symptom: motor spins, full-rate 1262-byte packets stream,
PandarView/driver decode structure, azimuths sweep — **every range and
intensity byte is 0x0000.** Point clouds collapse to a blob at the origin.

Root cause chain (reconstructed):
- Device Log JSON → `AzimuthFov` holds per-laser arrays: `laser_enable[40]`
  and `laser_range[40]`.
- **Saving the Azimuth FOV web page can persist these arrays in broken
  states.** Observed states: `laser_enable` all-0 (lasers disabled → zero
  ranges at full packet rate), and after a per-channel save of default rows:
  enable all-1 but `laser_range` all `[0,0]` (zero-width windows → packet
  output stops almost entirely; lone malformed 1180-byte zero frames at boot).
- The per-channel page loads rows as enabled/0.0–0.0 — **saving it as-loaded
  muzzles the sensor.**

### Diagnosis
Web console → Device Log (JSON) → `AzimuthFov`:
- `laser_enable` must be all **1**
- `laser_range` must be all **[0,3600]** (tenths of a degree)
Also check tcpdump: `sudo tcpdump -i <iface> udp port 2368 -c 1 -X` — real
data shows non-zero bytes between the `ffee` block markers.

### Fix that worked
**Factory reset from the web console**, then re-apply minimal settings.
Manual per-channel repair (all rows 0→360, Save, power cycle) writes the
arrays but did not restore ranging in our case; factory reset did,
immediately (verified in PandarView, then ROS).

### Rules going forward
- **Never press Save on the Azimuth FOV page.** Look with eyes only.
- After any config change, re-pull Device Log JSON and verify the two arrays.
- Power cycle after config changes; verify with tcpdump before blaming
  software.

## 3. NoiseFiltering observation
The broken (zero-ranges) state had `NoiseFiltering: 1`; factory default is
**0 (off)**. Causal role unproven (reset cleared everything at once), but:
**leave it OFF** — raw returns are what the SLAM/offline pipeline wants;
outlier filtering happens in post where it's visible and tunable.

## 4. Safe operating posture
Factory defaults + minimum deliberate changes only:
- IP 192.168.1.201 (factory), host 192.168.1.100/24
- Spin rate 600 rpm, Return Mode: note default after reset; Strongest is the
  clean indoor choice, Dual for vegetation
- Clock Source: GPS default is fine until ptp4l is deployed; then PTP
- Everything else: untouched. Every optional toggle is guilty until proven
  innocent (FOV page: convicted; NoiseFiltering: under indictment).

## 5. Other identifiers / facts
- Model string over PTC/web: **PA40-Zoox** (not "Pandar40P") — driver PTC
  auto-detection still works (correction file loads from lidar over PTC).
- Config placeholders in HesaiLidar_ROS_2.0 config.yaml must be emptied
  (`""`): multicast_ip_address, firetimes_path, channel fov filter path —
  literal placeholder strings cause ERROR/FATAL log lines.
- Stray 1180-byte all-zero UDP frames can appear at boot ("Packet with
  invaild delimiter" in driver log) — harmlessly rejected once ranging works.
- Teardown findings (unit #1): fiber-per-laser emitter tower, Artix-7
  XA7A100T on rotor, Zynq XA7Z020 + Micron DDR3 on stator, BCM89811 PHY.
  See [teardown/README.md](teardown/README.md).

## 6. Known-good end state (2026-07-26)
Factory reset → orange pair → BUELEC(100M, Master) → laptop 192.168.1.100 →
HesaiLidar_ROS_2.0 → `/lidar_points` at 10 Hz, 144k pts/frame, real ranges —
rings in RViz, dog on couch resolvable, ceiling fan visible frame-to-frame.
