# CLAUDE.md — Pandar40P Mobile Mapping Rig

Context file for Claude Code. This project has a long chat history behind it;
this file is the handoff. Read it fully before making changes.

## Who / what / where

- Matt, Rushville IN. Hands-on builder: welding, 3D printing, electronics,
  automotive diagnostics. Verify-everything-against-primary-sources doctrine —
  never assume pin assignments, register values, API behavior, or frame
  conventions; check the datasheet/manual/source. This doctrine has been
  vindicated repeatedly on this project.
- Repo: https://github.com/evilgenius79/pandar-40p (until now maintained by
  web upload; Claude Code takes over from here).
- Rig laptop: hostname `Lidar-Scanner`, user `lidar`. i7-12650H, RTX 4060
  8 GB, 32 GB RAM, dual-boot Ubuntu 22.04 + ROS 2 Humble. Workspace at
  `~/ros2_ws`. NIC pinned to 192.168.1.100/24.
- **The project WORKS as of 2026-07-30**: FAST-LIO2 produces a metrically
  sound multi-room indoor map from a hand-carried pass. Doorway measured
  0.77 m vs 0.813 m nominal. First successful bag:
  `~/bags/run_20260729_215544` (87 s, 871 lidar frames, 17,530 IMU).

## Hardware (all verified working)

- **Hesai Pandar40P** — Zoox robotaxi fleet pull, fw 2.20.17, $149 eBay.
  100BASE-T1 automotive Ethernet (BCM89811 PHY), NOT standard TX. Bridged by
  a BUELEC 100/1000Base-T1-TX-E converter: rate=100M, mode=MASTER, lidar
  ORANGE pair (Lemo pins 7/8) to terminal block. Web console 192.168.1.201,
  UDP 2368, 600 rpm, 1262-byte legacy P40 packets.
  Mounted plug-AFT on a welded mast, tilted forward (measured 58.2° from
  vertical by gravity, not the ~45° originally planned).
- **IMU** — ICM-42688-P on XIAO ESP32-S3 (SPI: D10 MOSI, D9 MISO, D8 SCK,
  D7 CS, D6 INT1). ±1000 dps / ±8 g, 200 Hz ODR (reg 0x27 per DS-000347;
  measured ~201 Hz). Scale: 4096 LSB/g, 32.8 LSB/dps. Publishes rad/s and
  m/s² (verified: peak gyro 0.5–0.8 walking, |a| 10.02).
  **Physically co-mounted under the lidar, axes parallel to the lidar's.**
  Gravity check accepted: ax +0.165, ay +8.392, az +5.194 (~1° yaw residual).
- **GPS** — u-blox M10 via XIAO (TX→D5, PPS→D4). ~1 Hz NO_FIX indoors
  (correct). Historical PPS flood quirk (~840/s without fix); the last bag
  recorded 0 PPS messages — unexplained change, watch-item.
- **Cameras** — 2× ELP-USB3DGS1200P01-H120 dual-lens global shutter
  (OG02B10, 3200×1200, USB2 UVC, MJPEG-always). Mounted, NOT aimed/bonded/
  calibrated yet. Calibrate LAST, after aim is final.
- Bridge serial: XIAO must be plugged in BEFORE launch — bridge node
  hardcodes `/dev/ttyACM0`. udev symlink is a wanted nicety.

## Coordinate frames (hard-won, do not re-derive casually)

- Lidar (Hesai manual Fig 2/3): Z = rotation axis up; Y points at the cable
  connector (azimuth 0°); X completes right-handed.
- This rig: plug aft ⇒ lidar +Y aft, +Z up the tilted spin axis, +X LEFT.
- IMU board silkscreen (component side up): +X right edge, +Y top edge away
  from pin header, +Z out of the component face.
- Mount recipe: board +Y at the plug, +Z along spin axis ⇒ +X lands left,
  matching the lidar. Identity extrinsic is true by construction.
- "Forward" is irrelevant to FAST-LIO2; only IMU↔lidar agreement matters.
  Never smuggle a display yaw into extrinsic_R.

## Software state

- `~/ros2_ws` packages: HesaiLidar_ROS_2.0 (working; /lidar_points 10 Hz,
  144k pts/frame, fields x,y,z,intensity,ring uint16, timestamp float64
  ABSOLUTE epoch seconds), livox_ros_driver2 (built with
  `-DROS_EDITION=ROS2`; NEVER run its build.sh — it broke the hesai build
  once), FAST_LIO on the ROS2 branch (patched, see below).
- Hesai driver config (`HesaiLidar_ROS_2.0/config/config.yaml`):
  - line 52 `use_timestamp_type: 1` (host receive time). Type 0 uses the
    sensor clock, which free-runs from Y2K epoch without GPS fix — that
    mismatch silently starves FAST-LIO2 forever. PTP is the later upgrade
    that makes type 0 correct (`ptp4l` master + console Clock Source→PTP).
  - line 15 firetimes path set to the real Pandar40P firetime CSV.
  - placeholders must be `""` (multicast_ip_address, channel fov path).
- **FAST-LIO2 local patches (in repo `patches/fastlio_pandar40p.patch`):**
  1. `src/preprocess.h` ~76: `float time;` → `double time;` and macro
     `(float, time, time)` → `(double, time, timestamp)`. Float would
     quantize epoch values to ~128 s steps — double is mandatory.
  2. `src/preprocess.cpp` in `velodyne_handler`, after the empty-cloud
     guard: subtract `points[0].time` from all per-point times, guarded by
     `if (pl_orig.points[0].time > 1e8)`. Without this, curvature becomes
     ~1.785e12 ms, lidar_end_time lands ~56,000 years out, and
     sync_packages never fires — totally silent failure.
- FAST-LIO2 config `~/ros2_ws/src/FAST_LIO/config/pandar40p.yaml`:
  lid_topic /lidar_points, imu_topic /imu/data_raw, time_sync_en false,
  lidar_type 2, scan_line 40, timestamp_unit 0 (⇒ time_unit_scale 1.e3,
  correct), blind 0.5, point_filter_num 3, filter sizes 0.5,
  extrinsic_T [0,0,0.07] (7 cm tape-measured, IMU→optical origin up the
  spin axis), extrinsic_R identity (true by construction),
  extrinsic_est_en true (now correctly polishing a small residual).
- **Launch the mapper with an ABSOLUTE config path** — relative paths
  resolve against the install tree and silently load the wrong file.
  Proof of correct load: console prints `p_pre->lidar_type 2`.
- Rig launch: `~/Desktop/rig_launch_v2.py` (repo `launch/rig.launch.py`) =
  hesai driver + its RViz + sensor bridge + optional `record:=true` bag.
- Recording protocol: hold dead still 3–5 s at start (gravity/bias init).

## Known landmines

- **Zero-ranges trap**: NEVER press Save on the web console's Azimuth FOV
  page. It can persist laser_enable all-0 / zero-width laser_range windows
  ⇒ spinning+streaming with all ranges 0x0000. Manual repair failed;
  factory reset fixed it. Verify via Device Log JSON: laser_enable all-1,
  laser_range all-[0,3600]. NoiseFiltering stays 0.
- **`ros2` CLI is unreliable on this machine**: stale /dev/shm fastrtps
  lock files (`rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` after
  pkill), daemon dies with `!rclpy.ok()` (`ros2 daemon stop && start`),
  `topic hz` blocks so chained commands never run. Prefer the standalone
  rclpy scripts in `scripts/diagnostics/` for ALL topic inspection.
- **FAST-LIO2 prints almost nothing when healthy.** A quiet console is not
  failure. RViz preset shows only mapper outputs (Fixed Frame camera_init
  doesn't exist until the mapper publishes TF ⇒ black screen ≠ broken).
  Verify with `scripts/diagnostics/scan_peek.py` and an /Odometry probe.
- **FAST-LIO2's `pcd_save_en` produced nothing** (PCD/ contains only the
  repo's 2-byte git placeholder named `1`). Export maps with
  `scripts/diagnostics/save_map.py [voxel] [out.pcd]` instead: start it
  before bag playback, Ctrl+C it (NOT the mapper) after playback ends.
  Reference: 25.7M raw → 262k @ 5 cm → 1.88M @ 2 cm.
- `ros2 bag info` Start/End come from the recorder's clock, not
  header.stamp — it cannot validate timestamp domains. Echo header stamps.
- 7/25 bags (~20 GB) are zero-ranges garbage, deletable. 7/26 bag has the
  Y2K/2026 split and is unusable for SLAM.
- CloudCompare snap can't drive the iGPU (0xa7a8 unsupported by its Mesa);
  it runs anyway (software or NVIDIA offload:
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`).
  PCDs from save_map.py are XYZ-only ⇒ render white; use Edit → Colors →
  Height Ramp.

## Debugging history in one paragraph

Five sequential blockers, none self-announcing: (1) 100BASE-T1 — "no link"
on a normal NIC looks like a dead unit; (2) the zero-ranges trap; (3) the
Y2K-vs-2026 timestamp domain split; (4) absolute per-point timestamps read
as offsets (the 56,000-year scan); (5) identity extrinsic against a tilted
lidar with a flat IMU — clean map stationary, divergent "whip" on rotation.
(5) was fixed by physically co-mounting the IMU, not by modelling the
rotation. Gyro units (rad/s vs deg/s) were suspected for the whip and ruled
out by measurement first. Full detail: docs/fastlio_setup.md and
docs/imu_extrinsic.md.

## Next steps (agreed order)

1. Compare `extrinsic_est_en`'s converged extrinsic to the [0,0,0.07]
   hand measurement across runs; adopt if stable.
2. `allan_variance_ros` overnight → real IMU noise params in the config.
3. Camera work: aim → 3M panel-bond → sharpie witness marks → intrinsics
   (checkerboard) → Koide direct_visual_lidar_calibration. One lens per
   board, ±30–35° splay, 10–15° up-pitch. No hardware trigger found (ELP
   email pending); MJPEG-always rule.
4. Stroller acquisition + mast-to-stroller build.
5. PTP time sync; revert use_timestamp_type to 0.
6. Longer outdoor capture with a closed loop to quantify drift.
7. Offline chain: GLIM (humble CUDA binaries) → HBA (Docker) → ERASOR
   dynamic removal → colorize → PINGS/Gaussian-LIC2 splats. 8 GB VRAM ⇒
   chunk scenes. Tier 2 later: ZED-F9P RTK via Indiana InCORS NTRIP (free).

## Working style with Matt

- Verify against primary sources before asserting; prefer measurements over
  inference and say which one a claim rests on. When a hypothesis is ruled
  out by data, retract it explicitly.
- One concrete next step at a time when debugging; small standalone scripts
  over CLI incantations; paste-ready commands with absolute paths.
- Diff proposed doc changes against the live repo rather than assuming a
  rewrite wins. Matt handles image links himself.
- Push straight to main (standing permission, 2026-07-31) — no PRs or
  side branches unless Matt asks.
- The dog is Bobo. He has appeared in both the first camera image and the
  first point cloud, and has a standing role as scan-quality control.
