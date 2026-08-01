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
  sound multi-room indoor map from a hand-carried pass. **Scale verified
  2026-08-01**, floor→ceiling from histogram peaks of the two horizontal
  surfaces against a 3.0607 m (10 ft 0½ in) tape measurement, averaged
  over histogram bin widths 2–15 mm so the method noise is known:

  | | measured | error |
  |---|---|---|
  | before the QoS fix | 3.0018 ± 0.0030 m | −1.92 % |
  | **after the QoS fix** | **3.0445 ± 0.0035 m** | **−0.53 %** |

  The +42.7 mm improvement is 12× the ±3.5 mm method noise, so recovering
  the dropped frames measurably improved metric accuracy. There is no
  global scale error. (An earlier note here said −1.33 %; that came from a
  single bin width and a gravity vector belonging to a different run.)
- **Bags — which one is good for what:**
  - `~/bags/run_20260801_014240` (67 s, 678 lidar frames, 13,651 IMU) —
    **the first post-reseat bag and the current reference.** `bag_grav.py`
    gives ax +0.409, ay +6.937, az +7.040, |a| 9.892, peak |gyro| 0.024
    rad/s, +Z 44.6° from gravity — `RESEATED CO-MOUNT`. This is the bag
    that validated the reseat (see "Extrinsic estimator" below). Use it
    for anything extrinsic. Map exported to `~/map_run_20260801_014240.pcd`
    (1.47 M points @ 2 cm).
  - Everything below predates the 2026-07-31 reseat and carries 14.4° of
    undeclared roll error; fine for SLAM/timestamp/pipeline work, void for
    adopting an extrinsic.
  - `~/bags/run_20260730_221408` (64 s, 642 lidar frames, 12,940 IMU) —
    the bag behind the known-good multi-room map, and the only bag whose
    mount was verified by measurement rather than by label:
    `scripts/diagnostics/bag_grav.py` over its first 5 s gives ax +0.137,
    ay +8.459, az +5.100, |a| 9.878, peak |gyro| 0.024 rad/s (genuinely
    still), +Z tilted 58.9° from gravity. What that measurement pinned is
    now known to be a **~14° crooked IMU**, not the mast — see "IMU reseat"
    below. Still the bag to use for SLAM, timestamp and pipeline work;
    useless for extrinsic adoption.
  - `run_20260729_215544` (87 s, 871 lidar frames, 17,530 IMU) — the
    first bag that mapped, and the one the timestamp-domain work was done
    against. It predates the IMU remount, so it was only ever valid for
    timestamp work, not SLAM or extrinsic work. **Deleted by Matt on
    2026-07-31** as no longer needed. Its mount was never verified by
    measurement and now cannot be; if the pre-/post-remount boundary ever
    matters again, note that the crooked-era gravity check recorded
    below (+0.165 / +8.392 / +5.194) is not itself dated.

## Hardware (all verified working)

- **Hesai Pandar40P** — Zoox robotaxi fleet pull, fw 2.20.17, $149 eBay.
  100BASE-T1 automotive Ethernet (BCM89811 PHY), NOT standard TX. Bridged by
  a BUELEC 100/1000Base-T1-TX-E converter: rate=100M, mode=MASTER, lidar
  ORANGE pair (Lemo pins 7/8) to terminal block. Web console 192.168.1.201,
  UDP 2368, 600 rpm, 1262-byte legacy P40 packets.
  **Return mode is DUAL — confirmed from data 2026-08-01**, which resolves
  the old "144k points/frame" puzzle. A recorded frame has 40 rings ×
  3,610 points = 144,400, and the busiest ring carries 3,610 points over
  **1,805 distinct azimuths, a ratio of exactly 2.00**. So azimuth
  resolution is the spec 0.2° at 10 Hz and the doubling is two returns per
  firing, not finer sampling. Consequences: it doubles the ~37.6 MB/s
  stream (and so contributed to the QoS frame drop), and FAST-LIO2 has no
  concept of return number — it treats a weak second return as an ordinary
  surface point, so partial hits on edges become spurious geometry. Dual
  return earns its keep outdoors seeing past vegetation; indoors it is
  mostly edge noise at double the bandwidth. Not yet A/B tested.
  Confirmed independently from the console 2026-08-01: `lidar_mode` = 2.
  (An earlier note here claimed dual return "contributed to the QoS frame
  drop". That was an assumption, not a measurement, and the probe data
  argues against it — BEST_EFFORT depth 5 received only 45 of 678 frames,
  6.6%, so halving the byte rate would not have rescued it.)
  Mounted plug-AFT on a welded mast, tilted forward **~45–47° from
  vertical** — Klein gauge on the mount plate reads ~47°, post-reseat
  gravity puts IMU +Z 44.5° from vertical, and the mast was built to the
  ~45° originally planned. The old "~58°, gravity-derived" figure was the
  crooked IMU talking, not the mast; corrected 2026-07-31. Don't write it
  with a decimal — the accel bias is uncalibrated until the
  allan_variance_ros run, so tenths overstate the precision.
- **IMU** — ICM-42688-P on XIAO ESP32-S3 (SPI: D10 MOSI, D9 MISO, D8 SCK,
  D7 CS, D6 INT1). ±1000 dps / ±8 g, 200 Hz ODR (reg 0x27 per DS-000347;
  measured ~201 Hz). Scale: 4096 LSB/g, 32.8 LSB/dps. Publishes rad/s and
  m/s² (verified: peak gyro 0.5–0.8 walking, |a| 10.02).
  **Physically co-mounted under the lidar, axes parallel to the lidar's.**
  **Reseated and rewired 2026-07-31; alignment validated 2026-08-01** by
  the estimator landing at roll +0.64°, pitch +0.38°, yaw −0.26°. Bench
  gravity check: ax +0.273, ay +6.938, az +7.062, |a| 9.904, +Z 44.5°.
  On the 8/01 bag: ax +0.409, ay +6.937, az +7.040, +Z 44.6°.
  Superseded crooked-mount check, kept because the older bags were
  recorded under it: ax +0.165, ay +8.392, az +5.194, +Z 58.9°.
  **Watch-item — the ax-derived yaw residual is creeping**: 0.8° (7/30
  bag) → 1.6° (7/31 bench) → 2.4° (8/01 bag). The estimator disagrees,
  putting yaw at −0.26°. Prime suspect is uncalibrated accel X bias:
  0.04 g would explain the whole gap, and gravity-derived yaw is the
  weakest check on the rig since gravity cannot see rotation about
  itself. `allan_variance_ros` (next step 2) settles it.
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
  matching the lidar. Identity extrinsic is true by construction — **but
  only if the board is actually mounted that way.** It was 14.4° off about
  X from the first co-mount until the 2026-07-31 reseat, and "by
  construction" hid it for a week. The claim is an assertion about
  workmanship, not a proof; re-verify with gravity after any mount work.
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
  - `multicast_ip_address` must be `""` (it is passed straight to
    SocketSource).
  - `channel_fov_filter_path` keeps the stock placeholder string in the
    working config — verified harmless. `ParseChannelFovFilterPath`
    (`libhesai/lidar_types.h` ~253) returns -1 with an empty filter map
    both for `""` and for a nonexistent path; the only difference is one
    cosmetic `LogError` at startup.
  - live copy of this file is committed at `ros2/config/hesai_config.yaml`.
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
  extrinsic_T [-0.057,-0.023,0.047] (tape-measured 2026-08-01 after the
  reseat: IMU sits 5.7 cm +X/left, 2.3 cm +Y/aft, 4.7 cm below the lidar
  centre; extrinsic_T is the lidar in IMU axes, so it is the negation —
  convention verified in `laserMapping.cpp:895` → `IMU_Processing.hpp:327`,
  which applies `R*p_lidar + T`), extrinsic_R identity (true by
  construction),
  extrinsic_est_en true — but see "Extrinsic estimator" below: it is NOT
  polishing a small residual, it roams several degrees in rotation. Left
  true anyway (Matt's call, 2026-07-31) because that is the configuration
  the known-good multi-room map was made with.
- **Launch the mapper with an ABSOLUTE config path** — relative paths
  resolve against the install tree and silently load the wrong file.
  Proof of correct load: console prints `p_pre->lidar_type 2`.
- Rig launch: `~/Desktop/rig_launch_v2.py` (repo `launch/rig.launch.py`) =
  hesai driver + its RViz + sensor bridge + optional `record:=true` bag.
- Recording protocol: hold dead still 3–5 s at start (gravity/bias init).
- **Replay/export/analyse is one command**:
  `scripts/diagnostics/run_test.sh [bag] [voxel]` — preflight, mount
  signature, mapper, PCD export, extrinsic analysis, frame-drop count.
- **`docs/commands.md` is the command cheatsheet** — record, replay,
  diagnostics, CloudCompare, DDS unwedging, web-console warnings.

## Known landmines

- **The lidar motor injects a 10 Hz line straight into the IMU.** Measured
  2026-08-01 by FFT of 40 s of live `/imu/data_raw`: a line at 10.01 Hz
  at **104× the mean** with the lidar powered, which is exactly 600 rpm.
  Unplugging the lidar drops it to 0.9× (noise floor) and accel sd falls
  2–3×: x 0.0181→0.0053, y 0.0110→0.0050, z 0.0136→0.0061 m/s². Only
  unplugged does the IMU meet its own datasheet.
  - **Powering down the ROS driver does NOT stop this** — the Pandar40P
    spins whenever it has power, regardless of whether anything is
    listening. Standby mode or unplugging are the only options.
  - Consequence for tuning: the *operational* IMU noise is 2–3× the quiet
    Allan-variance numbers. That argues for keeping FAST-LIO's inflated
    covariances rather than substituting honest sensor figures — see the
    caveat `allan.py` prints.
- **Zero-ranges trap — CAUSE CONFIRMED 2026-08-01: it is
  `NoiseFiltering = 1`.** Reproduced on demand and fully reversible; the
  old "manual repair failed, factory reset fixed it" was a
  misattribution. Measured:

  | state | pts/frame | zero-range | median range |
  |---|---|---|---|
  | `NoiseFiltering=0` | 144,000 | 0.1 % | 1.78 m |
  | **`NoiseFiltering=1`** | 144,400 | **100.0 %** | 0.00 m |
  | back to `0` | 144,000 | 0.1 % | 1.78 m |

  With the filter on the unit spins, streams a full 144k-point cloud, and
  puts every point at the origin — nothing errors, which is why it reads
  as dead hardware. Recovery is one call, no reset needed (note the
  firmware's spelling, `noise_filtring`):

      curl -s "http://192.168.1.201/pandar.cgi?action=set&object=lidar_data&key=noise_filtring&value=0"

  The **Azimuth FOV page was probably never the culprit** — it inherited
  the blame because a factory reset cleared `NoiseFiltering` as a side
  effect. Matt's recollection was right and the docs were wrong. Still no
  reason to go pressing Save there, but its guilt is retracted.
  Full console reference: **docs/lidar_console.md**.
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
- Old bags are already in `~/.local/share/Trash/files/`, not in `~/bags`
  (which as of 2026-07-31 holds only `run_20260730_221408`). Trash totals
  ~142 GiB: **eight**
  7/25 bags ~70 GiB (zero-ranges era; none has a `metadata.yaml`, only the
  `.db3`, so `ros2 bag play` cannot open them without reconstruction), two
  7/26 bags ~6 GiB (the Y2K/2026 split, unusable for SLAM), three 7/27
  bags ~67 GiB (post-fix, metadata intact, not known to be garbage).
  Nothing purged — Matt's call 2026-07-31; disk is 19% used with 722 GB
  free, so there is no pressure. Earlier note here said "seven, ~20 GB";
  both numbers were wrong.
- ~~**Half to two-thirds of lidar frames never reach the mapper.**~~
  **SOLVED 2026-08-01 — it was the subscription QoS.** History: 483 of 871
  on the 7/29 bag (45% lost), 336 of 642 on the 7/30 bag (48%), 228 of 678
  on the 8/01 bag (66%). Measured directly by running two probe
  subscriptions alongside the real mapper over one replay of the 8/01 bag:

  | subscriber | frames of 678 |
  |---|---|
  | RELIABLE, depth 200 | **677** |
  | BEST_EFFORT, depth 5 (what the mapper used) | 45 |
  | the mapper itself | 115 |

  `rclcpp::SensorDataQoS()` at `laserMapping.cpp:927` is BEST_EFFORT depth
  5, carrying ~35 MB/s of PointCloud2, and was silently discarding most of
  it. Replaced with `rclcpp::QoS(rclcpp::KeepLast(100)).reliable()`.
  Playback offers RELIABLE (`metadata.yaml` `reliability: 1`) so the
  profiles are compatible — note that had the publisher been BEST_EFFORT,
  a RELIABLE subscriber would have received *nothing*, so check before
  copying this fix elsewhere. **After the fix: 674 of 678 processed
  (99.4%).** In the patch file. Everything measured on maps built before
  2026-08-01 used a third to a half of the recorded data.
- CloudCompare snap can't drive the iGPU (0xa7a8 unsupported by its Mesa);
  it runs anyway (software or NVIDIA offload:
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`).
  PCDs from save_map.py are XYZ-only ⇒ render white; use Edit → Colors →
  Height Ramp.

## First outdoor run — 2026-08-01, `run_20260801_144508`

234 m sidewalk loop on the jogger stroller (pneumatic tires), 251 s at
0.93 m/s, out 112 m and back to the start. 2,514 lidar frames, 50,554 IMU.
Map `~/map_run_20260801_144508_level.pcd`, 465 MB.

**Drift, in the gravity-levelled frame:**

| | |
|---|---|
| path length | 233.86 m |
| loop-closure gap | **1.277 m = 0.55 % of path** |
| — horizontal | 1.273 m |
| — vertical | **−0.102 m** |
| true vertical range | −0.39 .. +1.18 m |

0.55 % is a normal, healthy figure for FAST-LIO2, which has no loop
closure at all. 10 cm of vertical error over 234 m (0.04 %) is the
standout — gravity anchors that axis and it shows.

**Pneumatic tires answered the shock question: no isolators needed.**

| | peak \|accel\| | % of ±8 g |
|---|---|---|
| indoor, hand-carried | 11.74 m/s² | 15 % |
| sidewalk, air tires | **32.59 m/s²** | **42 %** |

Peak 3.3 g, and **zero** samples above even 50 % of full scale; gyro
peaked at 7 %. Rubber isolator mounts are not needed — decision closed.

**Frame accounting held at 99.8 %** (2,508 of 2,514) under 4× the data of
the indoor run, so the QoS fix scales.

**Mount signature:** tilt 45.6°, peak \|gyro\| 0.0123 rad/s at init.
Residual yaw from ax was **−0.2°**, against 0.8 / 1.6 / 2.4° on earlier
power-ups — more support for turn-on bias rather than a moving mount.

### Two things this run flagged

- **The extrinsic estimator wanders on long dynamic runs.** Indoors its
  last-quarter sd was 0.004–0.024°; here it is 0.026–0.386°, with pitch
  drifting +0.96° and yaw +1.08° across the last quarter alone. T moved
  22 / 17 / 18 mm away from the declared value, monotonically rather than
  settling. Nothing is large in absolute terms, but it is drift, not
  convergence. **This is the evidence that finally justifies the
  `extrinsic_est_en: false` A/B** proposed on 2026-07-31 — replay this bag
  both ways and compare loop closure.
- **GPS went silent outdoors.** `/gps/fix` recorded **0 messages** over
  251 s outdoors, where indoors it managed 1 Hz NO_FIX. Meanwhile
  `/gps/pps` flooded 1,809 messages into 2.6 s (~687 Hz) — the documented
  PPS-flood quirk, back again. Does not affect SLAM, which ignores GPS,
  but it blocks next-step 7 (PTP/GPS timing) and wants investigating
  before then.

## The "doorway error" was probably never an error (2026-08-01)

The long-standing "0.77 m vs 0.813 m nominal" residual looks like a wrong
nominal rather than a mapping fault. Three things say so:

- **Scale is fine.** Floor→ceiling reads 3.0200 m against 3.0607 m taped
  (10 ft 0½ in): −1.33 % over 3 m. A map genuinely 12 % small would have
  put that ceiling at 2.69 m. Errors that are 1.3 % on one baseline and
  12 % on another are not scale errors — scale is proportional by
  definition.
- **Every doorway number so far was measured in a 46.5°-tilted view.**
  The map is stored in `camera_init`, the IMU's orientation at t=0, and
  the IMU rides a ~45° mast. CloudCompare's "Top" was not a plan view and
  Height Ramp shaded along a tilted axis. Picking two jamb faces in that
  view is unreliable at the 10 % level. Both 0.77 and 0.715 came from it.
- **0.813 m is a 32-inch door; the measurements say 28-inch.** Matt's
  0.715 m reading is 28.15 in against a 28 in slab at 0.711 m — a 0.15 in
  match. Objective measurement of the jamb faces in the levelled plan
  gives 0.631 m clear, and the widest run with no returns at all is
  0.571 m, both consistent with a 28 in slab in a stopped frame.
  **Caveat found 2026-08-01:** those gap figures came from the pre-QoS-fix
  map. On the denser post-fix map the "widest empty run" heuristic breaks
  down — three click pairs across the same doorway disagree by 651 mm,
  because returns now land *inside* the opening (door face, jamb reveal)
  and there is no longer a clean empty span. The heuristic depended on
  sparsity. Ceiling height remains the trustworthy scale check; settle the
  doorway with a tape, not with the point cloud.

**Open, low priority: put a tape on that actual doorway.** Until then
0.813 m is an unverified assumption that has been treated as ground truth
for a week. Use `scripts/diagnostics/floorplan.py` for any future map
measurement — it levels by the logged gravity vector and finds edges from
the data instead of from where you click.

## IMU noise, measured (2026-08-01, 8.58 h static)

`~/bags/allan_20260801_031155`, 6.21 M samples at 201.1 Hz, lidar
**unplugged**, house asleep — hourly probes confirm az sd 0.0060–0.0078
throughout, so the whole capture is clean. Analysed with
`scripts/diagnostics/allan.py` (not `allan_variance_ros`, which is ROS 1
only). Plot at `~/allan.png`.

| | noise density | vs datasheet | bias instability | vs datasheet |
|---|---|---|---|---|
| gyro x | 1.98 mdps/√Hz | better | 2.27 °/hr | ~at spec |
| gyro y | 1.62 mdps/√Hz | better | 3.48 °/hr | 1.4× worse |
| gyro z | 1.82 mdps/√Hz | better | **7.07 °/hr** | **2.8× worse** |
| accel x | 42.1 µg/√Hz | better | 13.2 µg | |
| accel y | 41.2 µg/√Hz | better | 17.3 µg | |
| accel z | 52.3 µg/√Hz | better | **53.7 µg** | |

Datasheet: gyro 2.8 mdps/√Hz and ~2.5 °/hr, accel 70 µg/√Hz. **Every axis
beats the datasheet on white noise.** Bias instability is at or somewhat
worse than spec, with z the weak axis on both sensors.

- **Do NOT paste the derived covariances into the config.** Measured:
  `gyr_cov 9.9e-10`, `acc_cov 2.0e-7`, `b_gyr_cov 3.6e-12`,
  `b_acc_cov 1.6e-9`. The config runs FAST-LIO's defaults of 0.1 / 0.1 /
  1e-4 / 1e-4 — **five to eight orders of magnitude higher**. That gap is
  not an error to correct: the process noise absorbs un-modelled error
  (vibration, deskew residual, extrinsic residual), and the operational
  figure is also 2–3× the quiet one because the lidar spins while mapping.
  Treat any change as an A/B against a known-good bag.
- **Watch-item CLOSED: the "creeping ax yaw residual" is turn-on bias,
  not a mount problem.** `ax` across sessions read +0.137, +0.165, +0.273,
  +0.409 m/s² — a spread of 0.27, which is **2,083× the measured x-axis
  bias instability** of 0.130 mm/s². It cannot be drift. A ±20–40 mg
  zero-g offset, ordinary for this part and re-rolled on every power
  cycle, maps to ±1.15–2.29° of apparent yaw, covering the observed
  0.8–2.4° range exactly. So gravity-derived yaw carries ~±2° of
  irreducible uncertainty on this rig unless the accel is bias-calibrated
  by a tumble test. Do not read small changes in it as mount movement.

## IMU reseat 2026-07-31 — the old mount was 14° crooked

The IMU was reseated and rewired on the bench. Gravity before and after,
both from a genuinely-still window:

| | ax | ay | az | \|a\| | +Z from vertical |
|---|---|---|---|---|---|
| crooked (7/30 bag, first 5 s) | +0.137 | +8.459 | +5.100 | 9.878 | 58.9° |
| reseated (2026-07-31 bench) | +0.273 | +6.938 | +7.062 | 9.904 | 44.5° |

Rotating one gravity direction onto the other gives **14.44°, about an
axis that is 99.8 % +X** (Y 1.7 %, Z 5.6 %). So the reseat was essentially
a pure **roll** correction about the IMU's X axis — the tilt axis of this
rig.

The consequences are the important part:

- **The mast was never 58°.** Gravity measures the *IMU*, and the IMU was
  crooked. With it straight, gravity says 44.5° and the Klein gauge on the
  mount plate says ~47°, against a mast built for ~45°. Two independent
  instruments now agree; one crooked sensor disagreed with both.
- **Every bag on disk was recorded through a 14° extrinsic error**, while
  the config declared identity. `extrinsic_R` identity was false for all
  of them.
- **A fresh bag is required before any extrinsic is adopted.** Nothing on
  disk can be used for that, no matter how clean the map looked.
- Two caveats on the 14.44°: gravity pins only two of three DOF — it is
  blind to yaw about the gravity vector, so a yaw error could have been
  present before and after and would not show here. And the comparison
  assumes the lidar itself did not move during the reseat; only the IMU
  was touched, but that is workmanship again, not measurement.

### Forensic check against the estimator (run on the 7/30 bag)

Prediction going in: the converged `extrinsic_R` on `run_20260730_221408`
should show ~13–14° of pitch. Re-parsed from the existing 336-scan
`mat_out.txt` — same run already logged earlier on 2026-07-31, not a fresh
replay. **The prediction is half right, and the half it gets wrong matters.**

- **Axis: predicted correctly, named wrongly.** The geometric mismatch is
  a rotation about X — roll — not pitch. And roll is exactly where the
  estimator went: roll −6.116°, against pitch +1.120° and yaw +0.612°. The
  estimator moved in the one plane the reseat says was wrong. That is real,
  independent support that the old mount was crooked about X.
- **Magnitude: not confirmed.** 6.116° of the 14.44° available is **42 %**.
  The estimator recovered under half of a misalignment that was fully
  present in the data for the whole 64 s.
- Two readings, and one 64 s bag cannot separate them: (a) the estimator
  under-converges — roll slid monotonically from t ≈ 15 s and was still
  creeping −0.12° per quarter when the bag ended, so a longer run might
  have kept going toward −14°; (b) it is not cleanly absorbing the mount
  error at all, and −6.1° is partly bias or weak rotational observability.
- **This strengthens, not weakens, "do not adopt R."** −6.1° is now known
  to be neither the mount (14.4°) nor identity (0°). It is a number the
  filter stopped at.

## Extrinsic estimator — RESOLVED for R (2026-08-01)

`run_20260801_014240`, first bag on the reseated mount, 228 scans over
66.7 s. Last-quarter means (n = 57), against the pre-reseat run:

| | pre-reseat (7/30) | **post-reseat (8/01)** | sd (8/01) |
|---|---|---|---|
| R roll | −6.116° | **+0.644°** | 0.006° |
| R pitch | +1.120° | **+0.384°** | 0.007° |
| R yaw | +0.612° | **−0.262°** | 0.011° |
| T x | +0.00125 m | −0.05762 m | 0.17 mm |
| T y | +0.00073 m | −0.02214 m | 0.06 mm |
| T z | 0.06908 m | 0.04615 m | 0.09 mm |

- **The reseat is validated and the R question is answered.** Given
  straight geometry the estimator sits at identity within 0.65° on every
  axis instead of sliding 6° away. So the −6.1° was *the mount being
  crooked*, not the filter wandering — the two hypotheses that one bag
  could not separate on 2026-07-31. `extrinsic_est_en: true` behaves
  correctly when the declared extrinsic is true. **"Identity by
  construction" is now earned by measurement rather than asserted.**
- **T still tells you nothing.** It landed 0.9 mm from the declared
  value — but it only *moved* 0.9 mm from where it was initialized. Same
  inactivity as before, starting from a correct value this time. This run
  is *consistent with* T being right; it is not evidence for it. The
  decisive evidence remains the 7/30 run, where T was initialized ~6 cm
  wrong and still did not move. `analyze_ext.py` now prints this caveat
  automatically whenever T moves less than 2 mm.
- Do not re-adopt anything from the pre-reseat rows above; they are kept
  only as the record of what a 14.4° misalignment looks like from inside
  the filter.

### Method, and the pre-reseat run

**The numbers in this subsection are pre-reseat**, recorded through the
14° crooked mount. They stand as a measurement of what the estimator did,
not as a candidate extrinsic.

Run the bag through the mapper with `runtime_pos_log_enable: true`, which
writes `~/ros2_ws/src/FAST_LIO/Log/mat_out.txt`. Columns (from
`laserMapping.cpp:1103`): 0 t, 1-3 euler_cur, 4-6 pos, **7-9 ext_euler
(DEGREES — `SO3ToEuler` multiplies by 57.3, `use-ikfom.hpp:105`), 10-12
offset_T_L_I**, 13-15 vel, 16-18 bg, 19-21 ba, then grav and point count.
Parse with `scripts/diagnostics/analyze_ext.py <mat_out.txt>`.

Measured on `run_20260730_221408`, the only mount-verified bag — verified
as crooked, as it turned out. 336 scans logged over 63.9 s. Last-quarter
means (t ≥ 46.7 s, n = 84):

| | mean | sd | drift over last quarter |
|---|---|---|---|
| T x | +0.00125 m | 0.13 mm | +0.21 mm |
| T y | +0.00073 m | 0.16 mm | −0.47 mm |
| **T z** | **0.06908 m** | **0.19 mm** | +0.64 mm |
| R roll | −6.116° | 0.040° | −0.116° |
| R pitch | +1.120° | 0.026° | +0.039° |
| R yaw | +0.612° | 0.008° | +0.005° |

- ~~**Translation confirms the tape measure.**~~ **RETRACTED 2026-08-01.**
  It was written as: T z settles at 0.0691 against the hand-measured 0.070,
  T x and T y inside 1.3 mm of zero, so [0,0,0.07] is right. That reads
  agreement into what is actually **inactivity** — T never moved more than
  ~2 mm from the value it was initialized with. Two independent proofs:
  - The mount carried a known 14.4° roll error, which puts the lidar
    origin at **y ≈ +17.5 mm** in the crooked IMU frame (sin 14.44° ×
    70 mm). The estimator reported y = +0.7 mm. This one follows from the
    rotation alone and does not depend on anything else being unchanged.
  - The 2026-08-01 tape measurement puts the true lever arm at
    **[−0.057, −0.023, 0.047]**, 7.7 cm and mostly *lateral*, against a
    declared [0,0,0.07]. The estimator found none of the ~6 cm of x. (This
    one assumes the board did not move far during the reseat — weaker.)
  **T is effectively unobservable in this data.** A 0.19 mm standard
  deviation was the initialization sitting still, not convergence — the
  same trap as the R number, one line down.
- **Rotation lands at roll −6.1°, and that is NOT adoptable yet.** Within
  this run it looks like genuine convergence: identity until t≈15 s, an
  asymptotic slide to ≈ −6.2°, then a tight hold (sd 0.04°). But a single
  run cannot distinguish convergence from a slow one-way drift, and the
  roll was still creeping −0.12° per quarter at the end. There is no
  second post-co-mount bag to repeat it against, so the "adopt if stable
  across runs" test **has not actually been run**.
- A −6.1° roll would also contradict "identity by construction". **The
  reseat settled which way this went:** the mount really was off — 14.4°
  about X — and the estimator saw the right axis but only 42 % of the
  angle. So "do not treat −6.1° as a measurement of the mount" was the
  correct call, and is now demonstrated rather than suspected.
- **What would settle it:** a hand-carried bag **on the reseated mount**,
  replayed the same way. Now that the geometry is believed straight, the
  estimator should sit near identity. If it again slides to several
  degrees, that is the estimator wandering rather than the mount, and
  `extrinsic_est_en: false` is the fix.
- Open question: the roaming R is a live candidate for the residual map
  error (doorway 0.77 m vs 0.813 m nominal). Pinning it with
  `extrinsic_est_en: false` is the obvious A/B and was proposed on
  2026-07-31; Matt chose to leave it `true` for now, since true is what
  produced the known-good map. Re-run that A/B when there is a bag with a
  closed loop to score drift against.

## Debugging history in one paragraph

Five sequential blockers, none self-announcing: (1) 100BASE-T1 — "no link"
on a normal NIC looks like a dead unit; (2) the zero-ranges trap; (3) the
Y2K-vs-2026 timestamp domain split; (4) absolute per-point timestamps read
as offsets (the 56,000-year scan); (5) identity extrinsic against a tilted
lidar with a flat IMU — clean map stationary, divergent "whip" on rotation.
(5) was fixed by physically co-mounting the IMU, not by modelling the
rotation. Gyro units (rad/s vs deg/s) were suspected for the whip and ruled
out by measurement first. A sixth, found 2026-07-31: that co-mount was
itself **14.4° crooked about X**, undetected for a week because "identity
by construction" was treated as a proof and the resulting 58.9° gravity
reading was attributed to the mast instead of to the sensor. It surfaced
only when a Klein gauge on the mount plate disagreed with the IMU. Full
detail: docs/fastlio_setup.md and docs/imu_extrinsic.md.

## Next steps (agreed order)

0. ~~Capture a fresh bag on the reseated mount.~~ **DONE 2026-08-01** —
   `run_20260801_014240`. Mount signature confirmed, estimator validated.
1. Compare `extrinsic_est_en`'s converged extrinsic to the hand
   measurement; adopt if stable. **R RESOLVED 2026-08-01, T unresolved and
   probably unresolvable this way.** R comes out at identity within 0.65°
   on the straight mount, which both validates the reseat and clears
   `extrinsic_est_en: true`. T has never been observably estimated on this
   rig, so it rests on the tape measure alone
   ([-0.057,-0.023,0.047]) and no amount of further bags will change that
   — a deliberate perturbation test would be the only way to probe it, and
   it is not worth doing while translation error stays at the noise level.
2. ~~Measure the doorway.~~ **DONE 2026-08-01 — and it dissolved.** Scale
   verified against the ceiling to −1.33 % over 3 m; the doorway shortfall
   is very likely a 28-inch door being compared to a 32-inch nominal. See
   "The 'doorway error' was probably never an error" above. Residual task
   is a tape measure on the real doorway, low priority.
3. ~~Confirm or kill the frame-drop suspicion.~~ **DONE 2026-08-01 — it
   was real and it is fixed.** BEST_EFFORT depth 5 was dropping two-thirds
   of the cloud; reliable QoS takes it to 99.4%. See the landmine entry.
   Worth re-running any density-sensitive conclusion now that the mapper
   sees 3-6x more data — starting with a fresh map of the 8/01 bag.
4. ~~`allan_variance_ros` overnight.~~ **DONE 2026-08-01** — 8.58 h
   static capture, analysed with our own `allan.py`. See "IMU noise,
   measured" above. Outcome: every axis beats the datasheet on white
   noise; the config is deliberately left at FAST-LIO's defaults; and the
   creeping ax yaw residual is closed as turn-on bias. Remaining option,
   low priority: a tumble test to calibrate absolute accel bias, which
   Allan variance cannot measure — that is what would tighten the ±2°
   uncertainty on gravity-derived yaw.
5. Camera work: aim → 3M panel-bond → sharpie witness marks → intrinsics
   (checkerboard) → Koide direct_visual_lidar_calibration. One lens per
   board, ±30–35° splay, 10–15° up-pitch. No hardware trigger found (ELP
   email pending); MJPEG-always rule.
6. Stroller acquisition + mast-to-stroller build. **PARTIAL** — the
   2026-08-01 photos show the rig mounted on a wheeled walking frame, so
   the mast-to-chassis build exists. The target platform is a **jogger
   stroller specifically for its pneumatic tires**: small hard wheels
   transmit sharp shock over sidewalk joints, and the accelerometer has
   only 6.7× headroom to its ±8 g full scale against the 11.74 m/s² peak
   of an indoor hand-carried pass. Clipping the accel corrupts
   integration silently. Air tires are the cheapest fix; compliance
   between the chassis and the sensor head is the next one — never
   between the IMU and the lidar, which must stay rigid.
   **Rubber isolator mounts deferred by decision 2026-08-01**: measure the
   real shock first on an outdoor bag (peak |accel| and its FFT), then size
   isolators against it. Buying blind risks a mount whose natural frequency
   sits inside the disturbance band, where isolators amplify rather than
   damp, and rubber mounts loaded far below their rating stay effectively
   rigid and do nothing at all.
7. PTP time sync; revert use_timestamp_type to 0.
8. Longer outdoor capture with a closed loop to quantify drift.
9. Offline chain: GLIM (humble CUDA binaries) → HBA (Docker) → ERASOR
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
