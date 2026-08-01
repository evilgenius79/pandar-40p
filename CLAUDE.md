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
  0.77 m vs 0.813 m nominal.
- **Bags — which one is good for what. Every bag on disk predates the
  2026-07-31 IMU reseat, so NONE is valid for adopting an extrinsic.**
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
  **Reseated and rewired 2026-07-31.** Current gravity check: ax +0.273,
  ay +6.938, az +7.062, |a| 9.904, +Z 44.5° from vertical (~1.6° yaw
  residual from ax). Superseded crooked-mount check, kept because every
  bag on disk was recorded under it: ax +0.165, ay +8.392, az +5.194,
  +Z 58.9° from vertical.
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
- **~45% of lidar frames never reach the mapper on bag replay** — 483 of
  871 on the since-deleted 7/29 bag, 336 of 642 on the 7/30 bag (one
  `mat_out.txt` row = one processed scan). Not compute-bound: the mapper
  spends ~5 ms/frame against a 100 ms budget, and `standard_pcl_cbk`
  (`laserMapping.cpp:283`) has no drop logic. Prime suspect is the
  subscription QoS — `rclcpp::SensorDataQoS()` at `laserMapping.cpp:927`
  is BEST_EFFORT depth 5, carrying ~35 MB/s of PointCloud2. UNCONFIRMED;
  would have affected the original live run identically. Worth a
  reliable-QoS A/B before trusting any density-sensitive result.
- CloudCompare snap can't drive the iGPU (0xa7a8 unsupported by its Mesa);
  it runs anyway (software or NVIDIA offload:
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`).
  PCDs from save_map.py are XYZ-only ⇒ render white; use Edit → Colors →
  Height Ramp.

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

## Extrinsic estimator (measured 2026-07-31, next-step 1 — PARTIAL)

**All numbers in this section are pre-reseat**, recorded through the 14°
crooked mount described above. They stand as a measurement of what the
estimator did, not as a candidate extrinsic.

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

0. **Capture a fresh bag on the reseated mount.** Blocks everything
   extrinsic. Same protocol (dead still 3–5 s, hand-carried multi-room
   pass), then `bag_grav.py` to confirm the ~44.5° signature recorded and
   `analyze_ext.py` to see where the estimator settles. Expectation now
   that the geometry is straight: near identity. Nothing on disk can
   substitute — every existing bag carries the 14° error.
1. Compare `extrinsic_est_en`'s converged extrinsic to the hand
   measurement across runs; adopt if stable. **PARTIAL 2026-07-31, T claim
   retracted 2026-08-01** — see "Extrinsic estimator" above. Neither half
   survived: R is void (pre-reseat, and it found 42 % of the known error),
   and T was never actually estimated — it sat at its initialization. The
   config now carries the tape measurement
   [-0.057,-0.023,0.047] rather than anything the filter produced. On the
   next bag, check whether T *moves at all* before reading meaning into
   where it lands.
2. `allan_variance_ros` overnight → real IMU noise params in the config.
   Also settles the accel bias, which is why the mast tilt is written
   ~45–47° and not 44.5°.
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
