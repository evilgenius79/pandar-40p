# Pandar40P Mobile Mapping Rig

A DIY walk-around 3D mapping system: a surplus **Hesai Pandar40P** 40-channel lidar
(fleet pull, **100BASE-T1 automotive Ethernet variant**) on a tilted mast, pushed on a
jogging stroller, running real-time lidar-inertial SLAM on a gaming laptop, producing
dense colorized point clouds, meshes, and Gaussian splats.

> **The T1 discovery:** these fleet-surplus Pandar40Ps are NOT defective when they
> show "no link" on a normal NIC — they speak 100BASE-T1 (single-pair automotive
> Ethernet, Broadcom **BCM89811** PHY, confirmed by teardown). A ~$100 T1→TX media
> converter bridges them to any laptop. See [docs/t1_ethernet.md](docs/t1_ethernet.md).

![Inside the Pandar40P: the 40-channel fiber loom](docs/teardown/img/01_fiber_loom.jpg)
*Unit #1 gave its life for science — full teardown with photos in [docs/teardown/](docs/teardown/README.md).*

## System overview

```
  STROLLER MAST (~2 m, braced)                       LAPTOP (stroller seat)
  ┌─────────────────────────────┐                    i7-12650H · RTX 4060 · 32 GB
  │ Pandar40P (45° fwd tilt)    │── T1 pair ──► T1/TX converter ──► GbE NIC
  │ ICM-42688-P ── XIAO ESP32-S3│────────── USB ──► /imu/data_raw (200 Hz)
  │ 2× ELP OG02B10 GS cameras   │────────── USB ──► colorization frames
  │ u-blox M10 GNSS             │────────── USB ──► gpsd/chrony (+LIO GPS factor)
  └─────────────────────────────┘
  12 V tool battery ── fuse ── lidar only (~18 W)    Ubuntu 22.04 · ROS 2 Humble
                                                     FAST-LIO2 live · GLIM offline
```

## Quick start (bench)

1. Wire power: 12 V (9–48 V OK), 3 A fuse → red+gray = V+, black+gray/white = GND.
   **No power switch — it spins on connect. Remove the lens film first.**
2. Wire data: lidar T1 pair → converter terminal block → RJ45 → laptop.
3. `sudo scripts/network/setup_lidar_nic.sh <iface>` (sets 192.168.1.100/24)
4. `scripts/capture/find_lidar.sh <iface>` — confirms UDP on :2368 and reveals the
   sensor's actual IP (fleet units may not be at the 192.168.1.201 default).
5. Web control: `http://192.168.1.201` — save the Device Log JSON, set Clock
   Source = PTP, spin rate 600 rpm.
6. Full procedure: [docs/bench_test_checklist.md](docs/bench_test_checklist.md)

## Repo map

| Path | What |
|---|---|
| `docs/` | Build guide, bench checklist, pinout, T1 notes, **zoox_quirks.md (critical)**, teardown |
| `hardware/` | BOM + wiring references |
| `scripts/network/` | NIC setup, PTP master, lidar discovery |
| `scripts/capture/` | rosbag2 recording helpers |
| `scripts/postprocess/` | offline pipeline notes/stubs (GLIM → HBA → cleanup) |
| `firmware/imu_bridge/` | XIAO ESP32-S3 + ICM-42688-P USB timestamping bridge (PlatformIO) |
| `ros2/imu_bridge_node/` | laptop-side serial→`sensor_msgs/Imu` publisher |
| `ros2/config/` | Hesai driver + FAST-LIO2 starting configs |

## Status

- [x] Lidar #1: teardown (T1 PHY identified — BCM89811); parts donor
- [x] Lidar #2: bench-verified — ranging, rings in RViz (after factory reset;
      see docs/zoox_quirks.md — READ BEFORE CHANGING SETTINGS)
- [x] T1 media converter: BUELEC — working (100M, Master, orange pair)
- [x] IMU bridge: ICM-42688-P + XIAO v2 firmware — ~186 Hz verified, physics checked
- [x] One-command startup: launch/rig.launch.py (driver + bridge + optional record)
- [ ] First recorded bag (indoor room + carried lap)
- [ ] FAST-LIO2 config evening → first SLAM map
- [ ] Camera color confirmation + focus lock + calibration (after mount bonding)
- [ ] Mast + stroller build
- [ ] First walking capture

## Docs index

- [Build guide (full plan, quality tiers)](docs/build_guide.md)
- [Bench test checklist](docs/bench_test_checklist.md)
- [Cable pinout + Ethernet wiring](docs/pinout.md)
- [Automotive Ethernet / T1 notes](docs/t1_ethernet.md)
- [**Zoox fleet quirks & the zero-ranges trap**](docs/zoox_quirks.md)
- [Teardown notes (unit #1)](docs/teardown/README.md)
