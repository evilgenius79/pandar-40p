# Pandar40P Mobile Mapping Rig

A DIY walk-around 3D mapping system: a surplus **Hesai Pandar40P** 40-channel lidar
(fleet pull, **100BASE-T1 automotive Ethernet variant**) on a tilted mast, pushed on a
jogging stroller, running real-time lidar-inertial SLAM on a gaming laptop, producing
dense colorized point clouds, meshes, and Gaussian splats.

> **It maps.** As of 2026-07-30, FAST-LIO2 produces a metrically sound map of a
> multi-room interior from a hand-carried pass — a doorway measures 0.77 m against
> 0.813 m nominal. Getting there took five distinct fixes, none of which announced
> itself: see [docs/fastlio_setup.md](docs/fastlio_setup.md) and
> [docs/imu_extrinsic.md](docs/imu_extrinsic.md).

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
│ Pandar40P (fwd tilt)        │── T1 pair ──► T1/TX converter ──► GbE NIC
│ ICM-42688-P ── XIAO ESP32-S3│────────── USB ──► /imu/data_raw (200 Hz)
│   (co-mounted under lidar)  │
│ 2× ELP OG02B10 GS cameras   │────────── USB ──► colorization frames
│ u-blox M10 GNSS             │──── via XIAO ───► /gps/fix
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
5. Web control: `http://192.168.1.201`. Save the Device Log JSON and verify
`laser_enable` is all-1 and `laser_range` all-[0,3600]. **Never press Save on the
Azimuth FOV page** — see [docs/zoox_quirks.md](docs/zoox_quirks.md).
6. Full procedure: [docs/bench_test_checklist.md](docs/bench_test_checklist.md)

## Quick start (mapping)

Workspace built per [docs/fastlio_setup.md](docs/fastlio_setup.md) — the driver
config and two small FAST-LIO2 source patches there are **required**, not optional.
XIAO plugged in before launch (bridge hardcodes `/dev/ttyACM0`).

```bash
# record a run — hold still for the first 3–5 s (gravity/bias init)
ros2 launch launch/rig.launch.py record:=true

# replay through the mapper — absolute config path is required
ros2 launch fast_lio mapping.launch.py \
  config_file:=/home/lidar/ros2_ws/src/FAST_LIO/config/pandar40p.yaml
ros2 bag play ~/bags/<run_dir>

# export the map (FAST-LIO2's own pcd_save doesn't work here)
python3 scripts/diagnostics/save_map.py 0.02 ~/map.pcd
```

## Repo map

| Path                     | What                                                             |
| ------------------------ | ---------------------------------------------------------------- |
| `docs/`                  | Build guide, bench checklist, pinout, T1 notes, SLAM setup, extrinsic notes, teardown |
| `hardware/`              | BOM + wiring references                                          |
| `patches/`               | FAST-LIO2 source patches for the Hesai driver's point format     |
| `scripts/network/`       | NIC setup, PTP master, lidar discovery                           |
| `scripts/capture/`       | rosbag2 recording helpers                                        |
| `scripts/diagnostics/`   | rclpy topic-inspection tools + PCD export (the `ros2` CLI is unreliable on the rig laptop) |
| `scripts/postprocess/`   | offline pipeline notes/stubs (GLIM → HBA → cleanup)              |
| `firmware/imu_bridge/`   | XIAO ESP32-S3 + ICM-42688-P USB timestamping bridge (PlatformIO) |
| `ros2/imu_bridge_node/`  | laptop-side serial→`sensor_msgs/Imu` publisher                   |
| `ros2/config/`           | Hesai driver + FAST-LIO2 configs                                 |

## Status

- [x] Lidar #1: teardown (T1 PHY identified — BCM89811); parts donor
- [x] Lidar #2: bench test passed after factory reset (zero-ranges trap — see [docs/zoox_quirks.md](docs/zoox_quirks.md))
- [x] T1 media converter: BUELEC 100/1000Base-T1-TX-E — in service (100M / MASTER / orange pair)
- [x] IMU bridge: ICM-42688-P on XIAO ESP32-S3, 200 Hz, units verified (rad/s, m/s²)
- [x] IMU co-mounted under the lidar; alignment verified by gravity (~1° yaw residual)
- [x] Timestamp domains unified (`use_timestamp_type: 1`; PTP is the later upgrade)
- [x] FAST-LIO2 patched for the Hesai point format ([patches/](patches/))
- [x] **First indoor SLAM run — multi-room map, doorway 0.77 m vs 0.813 m nominal**
- [x] PCD export pipeline (`scripts/diagnostics/save_map.py`)
- [ ] Camera color confirmation + calibration
- [ ] Mast + stroller build
- [ ] Extrinsic refinement (compare `extrinsic_est_en` output to the 7 cm hand measurement)
- [ ] IMU noise characterization (`allan_variance_ros` overnight)
- [ ] PTP time sync, revert to sensor timestamps
- [ ] First outdoor capture with loop closure
- [ ] Offline chain: GLIM → HBA → dynamic removal → colorize

## Docs index

- [Build guide (full plan, quality tiers)](docs/build_guide.md)
- [Bench test checklist](docs/bench_test_checklist.md)
- [**FAST-LIO2 setup + debugging history**](docs/fastlio_setup.md)
- [**IMU mounting + extrinsic verification**](docs/imu_extrinsic.md)
- [Zoox fleet-variant quirks](docs/zoox_quirks.md)
- [Cable pinout + Ethernet wiring](docs/pinout.md)
- [Automotive Ethernet / T1 notes](docs/t1_ethernet.md)
- [Teardown notes (unit #1)](docs/teardown/README.md)
