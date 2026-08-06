# Session 2 summary (compressed) — 2026-07-29/30 "It Maps"

Compressed handoff summary of the second working conversation, **preserved
verbatim as a record of what was believed at the time**. This is the session
where SLAM was achieved: the absolute per-point-timestamp patch, the
extrinsic whip diagnosis, the IMU co-mount, and the metric verification.
Current truth lives in CLAUDE.md, docs/fastlio_setup.md, and
docs/imu_extrinsic.md.

Superseded since:

- **The IMU co-mount described here was 14.4° crooked about X.** It read as
  correct because "identity by construction" was treated as a proof rather
  than an assertion about workmanship. Reseated 2026-07-31.
- **The doorway metric verification is retracted** — the 0.813 m nominal was
  a 32-inch door against what the tape says is a 28-inch slab, and the
  measurement was taken in a ~46° tilted view. Scale is separately verified
  at −0.53 % against a taped ceiling.
- **Every map from this session used a third to a half of its own data**, to
  a BEST_EFFORT depth-5 subscription QoS bug fixed 2026-08-01.
- The bag-cleanup note below says "seven 7/25 bags (~20 GB)". It is **eight,
  ~70 GiB**, and nothing has been purged.

---

Compaction Summary — pandar40p-mobile-mapper (session of 2026-07-29/30)
Continues from: /mnt/transcripts/2026-07-27-18-31-50-pandar40p-mapper-build.txt (prior compaction). Matt, Rushville IN. Rig: Hesai Pandar40P (Zoox pull, 100BASE-T1 via BUELEC converter), ICM-42688-P on XIAO ESP32-S3, u-blox M10, gaming laptop (lidar@Lidar-Scanner), Ubuntu 22.04 + ROS 2 Humble + FAST-LIO2.

OUTCOME: SLAM WORKS. Metrically verified map achieved this session.

Blockers found and fixed (in order)

Timestamp domain fix confirmed — use_timestamp_type: 1 (line 52, Hesai config.yaml) took effect; new bag ~/bags/run_20260729_215544 (87.1 s, 871 lidar @10 Hz, 17,530 IMU @201 Hz, 88 GPS, 0 PPS — flood stopped unexplained, watch-item) has both topics in 2026 epoch. Note: ros2 bag info Start/End can't validate stamp domains (recorder clock, not header.stamp).

Absolute per-point timestamps (the big one). Measured: per-point timestamp = absolute epoch seconds, first == header, span 0.0997 s. FAST-LIO2's velodyne_handler (preprocess.cpp:296) sets given_offset_time=true since last time > 0, curvature = time × time_unit_scale (1.e3 for timestamp_unit 0) → 1.785e12 ms → lidar_end_time (laserMapping.cpp:406) ~56,000 years out → line 415 last_timestamp_imu < lidar_end_time false forever → sync_packages never fires. Silent (line 328 sync warning compares only header stamps). Patch applied & verified (lines 311/314): after empty-cloud guard, if (points[0].time > 1e8) subtract ts_base = points[0].time from all points. Prereq confirmed: preprocess.h:76 double time; (float would quantize epoch to ~128 s steps). Result: /Odometry published (first: x=-0.013 y=-0.002 z=0.007); /cloud_registered = 47.9k pts/scan (144k÷point_filter_num 3), extents ~15×8×3.5 m.

The whip — map clean stationary, diverged into whip on rotation (trajectory ran to 865 m view distance). Ruled out by measurement: gyro units OK (peak 0.52–0.79 rad/s walking), accel OK (10.02 m/s²). Cause: identity extrinsic vs tilted lidar + flat IMU; extrinsic_est_en can't converge from 45° off. Fix: physical co-mount, not rotation modeling. Matt remounted IMU under lidar.

Frames resolved (memory file updated): Hesai manual Fig 2/3 — lidar Z = spin axis up, Y toward cable connector (azimuth 0°), X completes RH. Rig mounts plug AFT ⇒ lidar +Y aft, +X left. IMU silkscreen: +X right edge, +Y top edge (away from header), +Z out of component face. Mount recipe: board +Y at plug, +Z along spin axis. (Matt's earlier "X fwd Y left" described the IMU, not lidar — was clarified. Also: a vendor quote "forward +X, right +Y, up +Z" is not right-handed; distrust prose, trust silkscreen/manual.) "Forward" is irrelevant to FAST-LIO2.

Gravity verification passed: ax +0.165 (≈1° yaw residual — the critical near-zero axis), ay +8.392, az +5.194, |a| 9.870 (0.7% scale, auto-normalized). Signs match plug-aft + forward tilt. Actual mast tilt = 58.2° from vertical, not the ~45° planned (harmless; noted). Lever arm ~7 cm up spin axis ⇒ config now: extrinsic_T [0,0,0.07], extrinsic_R identity, extrinsic_est_en: true (proper use now).

Result: whip gone. Multi-room map, planar walls, Bobo appears as coherent point-cloud blob. Doorway measured 0.77 m vs 0.813 m nominal in CloudCompare (point-pair tool; thin cross-section slab technique to avoid diagonals; long walls are better accuracy tests than doorways).

Infrastructure findings

ros2 CLI unreliable on this machine: stale /dev/shm/fastrtps_* locks (pkill then rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*), daemon death (!rclpy.ok() → ros2 daemon stop/start), topic hz blocks chained commands. Standard practice now: standalone rclpy scripts.

FAST-LIO2 prints ~nothing when healthy — quiet console ≠ failure (earlier misdirection acknowledged/corrected). RViz preset shows only mapper outputs; camera_init TF absent until mapper publishes ⇒ black ≠ broken. RViz nav: Decay Time on CloudRegistered to accumulate; height-ramp coloring.

pcd_save_en produces nothing (PCD/ has only upstream's 2-byte 1 placeholder). Export via ~/save_map.py [voxel] [out.pcd]: subscribes /cloud_registered, voxel-dedupes (np.unique on floored keys), writes binary PCD on Ctrl+C (Ctrl+C the accumulator, not mapper; downsample sits silent ~1–2 min). First exports: 25.67M raw → 262k @5 cm → 1.878M @2 cm (~/map.pcd, ~/map_2cm.pcd). PCDs are XYZ-only → white in CloudCompare → Edit→Colors→Height Ramp.

CloudCompare snap: iGPU 0xa7a8 unsupported by its Mesa (libGL iris errors harmless if window opens); NVIDIA offload via __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia.

Scripts on laptop (~/): scan_peek.py, imu_units.py, grav_vec.py, save_map.py, run_test.sh (one-shot checks+launch+replay; its trap SIGTERMs mapper — don't use for PCD export; edit BAG= line per run). Heredoc pastes into his terminal scramble — deliver scripts as downloadable files, mv ~/Downloads/... ~/.

Repo (github.com/evilgenius79/pandar-40p — fetched, real layout known)

Existing: docs/, firmware/imu_bridge, hardware/, ros2/, scripts/{network,capture,postprocess}, README with T1 callout + ASCII diagram + status checklist (was stale at "converter ordered"). Delivered repo-update-2026-07-30.zip (in outputs): rewritten README (matches his voice/structure; "It maps" callout; checklist current; fixed old "Clock Source = PTP" bench step), docs/fastlio_setup.md (full runbook + debugging history), docs/imu_extrinsic.md (whip story + gravity method), patches/fastlio_pandar40p.patch, scripts/diagnostics/ (5 scripts + README). Matt uploads via web, handles image links himself (map screenshot → README callout; Bobo → imu_extrinsic Result section). Caveat flagged: docs link docs/zoox_quirks.md which wasn't visible in repo listing — may 404 if only in his v3 zip. [Resolved 2026-07-31: zoox_quirks.md committed with the v3 sync.]

Also delivered CLAUDE.md (in outputs) for repo root — Matt is moving the project to Claude Code on the rig laptop; file carries full state, landmines, frames, patches, next-steps, working style. Reminder given: ROS workspace ~/ros2_ws is outside the repo; keep CLAUDE.md updated.

Next steps (agreed order)

1. Compare extrinsic_est_en converged value vs [0,0,0.07] across runs. 2. allan_variance_ros overnight. 3. Cameras: aim → panel-bond → witness marks → intrinsics → Koide calibration. 4. Stroller build. 5. PTP → revert use_timestamp_type 0. 6. Outdoor loop-closure capture. 7. Offline: GLIM → HBA → ERASOR → colorize → splats (8 GB VRAM ⇒ chunk). Cleanup pending: seven 7/25 zero-ranges bags (~20 GB) deletable.

Working style notes

Verify against primary sources; state measurement vs inference; retract explicitly when data kills a hypothesis (done twice this session: "map is a ribbon" misread, gyro-units theory). One concrete step at a time; paste-ready absolute paths; three-terminal test pattern (mapper / bag / probe). Dog: Bobo — appeared in first light and first map.
