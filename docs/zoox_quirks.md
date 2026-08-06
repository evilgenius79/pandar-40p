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

## 2. THE ZERO-RANGES TRAP — cause confirmed: `NoiseFiltering = 1`
The worst one. Symptom: motor spins, full-rate 1262-byte packets stream,
PandarView/driver decode structure, azimuths sweep — **every range and
intensity byte is 0x0000.** Point clouds collapse to a blob at the origin.
Nothing errors, which is exactly why it reads as dead hardware.

**Reproduced on demand 2026-08-01 and fully reversible.** Toggling one
setting moves the unit in and out of the fault:

| state | pts/frame | zero-range | median range |
|---|---|---|---|
| `NoiseFiltering=0` | 144,000 | 0.1 % | 1.78 m |
| **`NoiseFiltering=1`** | 144,400 | **100.0 %** | 0.00 m |
| back to `0` | 144,000 | 0.1 % | 1.78 m |

### Fix — one call, no factory reset
Mind the firmware's spelling, `noise_filtring`:

```bash
curl -s "http://192.168.1.201/pandar.cgi?action=set&object=lidar_data&key=noise_filtring&value=0"
```

### What this retracts
- **The Azimuth FOV page was probably never the culprit.** It inherited the
  blame because the factory reset that recovered the unit also cleared
  `NoiseFiltering` as a side effect — one action, two changes, and the wrong
  one got the credit. Matt's recollection that the FOV page was innocent was
  right and these docs were wrong. Still no reason to go pressing Save
  there, but that is general caution, not a known fault.
- **"Manual repair failed; only a factory reset fixed it" was a
  misattribution.** Manual repair failed because it was repairing the wrong
  setting.
- **The old diagnostic check was itself wrong** and false-alarms on a
  healthy lidar. It said to verify `laser_enable` all-1 and `laser_range`
  all-`[0,3600]`. Those per-laser arrays are only live when
  `angle_setting_method` is `1`. This unit runs method `0`, where the global
  `lidar_range` governs and **the per-laser arrays read all-zero normally**.

### Diagnosis, corrected
```bash
python3 scripts/diagnostics/lidar_config.py
```
It reads `angle_setting_method` first and reports which block is actually in
force, so an all-zero array is interpreted rather than panicked over. Check
`NoiseFiltering` before suspecting anything else. Independent confirmation
from the wire: `sudo tcpdump -i <iface> udp port 2368 -c 1 -X` — real data
shows non-zero bytes between the `ffee` block markers.

## 3. NoiseFiltering — leave it OFF
Factory default is **0 (off)**, and section 2 is why: `1` is the confirmed
cause of the zero-ranges fault on this unit. Beyond that, raw returns are
what the SLAM and offline pipeline want — outlier filtering belongs in post,
where it is visible and tunable.

## 4. Safe operating posture
Factory defaults + minimum deliberate changes only:
- IP 192.168.1.201 (factory), host 192.168.1.100/24
- Spin rate **600 rpm — settled**. 1200 rpm adds no data: the firing rate is
  fixed, so it halves azimuth resolution (0.2° → 0.4°) to double the frame
  rate. It earns its keep at vehicle speeds, not on a stroller.
- Return Mode: this unit runs **Dual** (`lidar_mode` = 2, confirmed from both
  the console and the data). Kept on for outdoor work — second returns punch
  through foliage, and the 2026-08-01 map shows internal canopy structure
  rather than a solid shell. Indoors it is mostly edge noise at double the
  bandwidth, since FAST-LIO2 has no concept of return number.
- Clock Source: GPS default is fine **only while `use_timestamp_type: 1`**.
  Read `docs/lidar_console.md` before touching it — switching the driver to
  sensor timestamps with the wrong clock source is a silent failure.
- Everything else: untouched. Every optional toggle is guilty until proven
  innocent — **`NoiseFiltering`: convicted on reproduction; FOV page:
  acquitted, see section 2.**

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
