# Session 1 summary (compressed) — through 2026-07-26 "First Light"

Compressed handoff summary of the first working conversation, preserved
verbatim. Some values were superseded by later work (tilt measured 58.2°
not 45°; IMU later co-mounted under the lidar; the 7/26 bag's Y2K/2026
timestamp split was fixed by `use_timestamp_type: 1`). Current truth lives
in CLAUDE.md, docs/fastlio_setup.md, and docs/imu_extrinsic.md.

---

Project: pandar40p-mobile-mapper (Matt, Rushville IN)
Goal: walk-around 3D mapping — Hesai Pandar40P on a welded stand/mast (45° forward tilt, ~2 m target height), pushed on a future jogging stroller, gaming laptop (i7-12650H, RTX 4060 8 GB, 32 GB, GbE, dual-boot Ubuntu 22.04 + ROS 2 Humble) running FAST-LIO2 live; offline chain later: GLIM → HBA → dynamic removal → colorize → meshes/Gaussian splats. Doctrine: record raw bags always; real-time for confidence, offline for quality.

Hardware state (all verified unless noted)

Lidar #2 "PA40-Zoox" (fw 2.20.17, s/n PA4038C35C9738C15F, $149 eBay RetradeInc, ~2h lifetime at arrival): WORKING after factory reset. 100BASE-T1 automotive Ethernet (Broadcom BCM89811 PHY) — NOT standard TX.

Lidar #1: torn down (never defective — was T1). Findings: fiber-per-laser emitter tower, Artix-7 XA7A100T on rotor, Zynq XA7Z020 + Micron DDR3 on stator, wedge driver boards, ADG658/AD8370 mux receive chain, HC4067 tx muxes, 905nm aspherics + dielectric mirrors salvageable. 12 photos committed to repo. Parts donor; seller didn't want it back.

BUELEC 100/1000Base-T1-TX-E converter: in service. Recipe: rate=100M, mode=MASTER, lidar ORANGE pair (Lemo 7/8) to terminal block; blue pair unused. Mounted on mast.

IMU: ICM-42688-P on XIAO ESP32-S3 (SPI: D10 MOSI, D9 MISO, D8 SCK, D7 CS, D6 INT1; GPS: M10 TX→D5, PPS→D4, wired and connected). v2 firmware flashed with datasheet-verified 0x27 registers (±1000dps/±8g, 200 Hz ODR; DS-000347: FS bits [7:5]=001, ODR bits [3:0]=0111). Measured ~186-201 Hz, physics verified (Z≈9.87 flat). MPU6050 retired unused. Scale constants: 4096 LSB/g, 32.8 LSB/dps.

Cameras ×2: ELP-USB3DGS1200P01-H120 dual-lens color GS (OG02B10, 3200×1200 side-by-side, USB2 UVC, MJPEG-always rule). On rig in printed cases on ball mounts (tubes on welded mount). Plan: one lens per board, ±30-35° splay, 10-15° up-pitch; aim → 3M panel-bond → sharpie witness marks → calibrate LAST. No external trigger pad found (email ELP pending). Separate USB ports.

GPS u-blox M10: wired to XIAO; indoors publishes ~1 Hz NO_FIX (correct). PPS quirk: ~840/sec flood without fix — watch-item, driveway test or u-center config later.

Power: bench 12V or tool battery + 3A fuse → lidar only (9-48V, ~18W); laptop self-powered. NIC permanent 192.168.1.100/24 via NetworkManager GUI.

Deferred: ZED-F9P RTK (Tier 2, free Indiana InCORS NTRIP), stroller acquisition, mast-to-stroller build.

Critical Zoox quirks (documented in docs/zoox_quirks.md)

T1 Ethernet: powers/spins but "no link" on normal NIC = needs converter, not defective.

Zero-ranges trap: Azimuth FOV web page saves can persist laser_enable[40]=all-0 or laser_range[40]=all-[0,0] (zero-width windows) → spinning + streaming but all range bytes 0x0000, or output stops. Per-channel page loads as enabled/0.0-0.0 — saving as-loaded muzzles sensor. Manual repair didn't restore ranging; factory reset did (verified PandarView then ROS). NEVER press Save on FOV page. Verify via Device Log JSON: laser_enable all-1, laser_range all-[0,3600].

NoiseFiltering was 1 in broken state, factory default 0 — leave OFF.

Driver config placeholders must be "" (multicast_ip_address, channel fov filter path). Firetimes path NOW SET to real file: /home/lidar/ros2_ws/src/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/firetime_correction/Pandar40P_Firetime Correction File.csv (line 15 of config.yaml).

Web console: 192.168.1.201, UDP 2368 broadcast, 600rpm, packets 1262B legacy P40 format (ffee markers). Stray 1180B zero frames at boot = harmless. Clock Source currently GPS (free-run → Y2K epoch timestamps).

Software state

Ubuntu 22.04 + ROS 2 Humble; workspace ~/ros2_ws with HesaiLidar_ROS_2.0 (working: /lidar_points 10 Hz, 144k pts/frame, frame_id hesai_lidar, fields x,y,z,intensity,ring(uint16),timestamp(float64 abs seconds); PTC auto-loads correction), livox_ros_driver2 (built with -DROS_EDITION=ROS2; never use its build.sh — it broke hesai build once; fixed by rebuilding hesai with no args), FAST_LIO on ROS2 branch (built clean).

First light achieved: rings in RViz, dog "Bobo" visible on couch, ceiling fan blades visible frame-to-frame.

Launch: ~/Desktop/rig_launch_v2.py (repo launch/rig.launch.py) = hesai driver (+its own RViz) + sensor bridge + record:=true bag. Gotcha: bridge hardcodes /dev/ttyACM0; XIAO must be plugged before launch.

Bag ~/bags/run_20260726_222044 — 68s, 682 lidar frames (10 Hz), 13,746 IMU (~201 Hz), 68 GPS fix, 57k PPS. FLAWED: lidar stamps are Y2K epoch (sec≈946688679, sensor free-run clock) while IMU is 2026 ROS time — FAST-LIO2 can never sync them. Will be superseded by re-record. Seven 7/25 bags are zero-ranges garbage (deletable, ~20 GB).

dialout group added; workspace sourcing in .bashrc.

FAST-LIO2 session — debug history & current state

Build: catkin error → ROS2 branch → needed livox_ros_driver2 → built with -DROS_EDITION=ROS2 → fast_lio Finished.

Config at ~/ros2_ws/src/FAST_LIO/config/pandar40p.yaml: lid_topic /lidar_points, imu_topic /imu/data_raw, time_sync_en false, lidar_type 2, scan_line 40, timestamp_unit 0, blind 0.5, identity extrinsic + extrinsic_est_en true (deliberate first-pass; real extrinsic = iteration 2; IMU stays flat-mounted — bookkeeping only). Must launch with ABSOLUTE path: ros2 launch fast_lio mapping.launch.py config_file:=/home/lidar/ros2_ws/src/FAST_LIO/config/pandar40p.yaml (install-tree lookup fails for new files; correct load shows "lidar_type 2").

"Failed to find match for field 'time'" FIXED: ~/ros2_ws/src/FAST_LIO/src/preprocess.h line 76 float time;→double time; and macro line ~81 (float, time, time)→(double, time, timestamp) (nano wraps lines; Esc+$ toggles). Rebuilt; node starts clean.

Then: topics showed 1 pub/1 sub each (mapper subscribed), but /cloud_registered & /Odometry silent; console printed only "lidar loop back, clear buffer" on replays. DIAGNOSIS: timestamp-domain mismatch (Y2K lidar vs 2026 IMU) — sync engine waits forever.

FIX IN FLIGHT (just instructed, awaiting user execution): Hesai config.yaml line 52 use_timestamp_type: 0 → 1 (host receive time; symlinked config, no rebuild). Trade-off: ms-scale receive jitter, fine to birth SLAM; PTP (ptp4l master + console Clock Source→PTP) is the later quality upgrade making type 0 correct. Per-point de-skew uses relative per-point timestamps — if de-skew misbehaves later, pre-briefed fix = normalize in preprocess.cpp (subtract scan-start).

NEXT STEPS (exact sequence): (1) restart rig launch with record:=true, rig powered; (2) sanity: ros2 topic echo /lidar_points --field header.stamp --once must show sec≈1785xxxxxx matching IMU; (3) re-record same take (room/fan ~60-90s + slow carried lap); (4) fresh mapper launch (absolute path) + ros2 bag play new bag; success = per-scan chatter, /Odometry ticking, RViz (fastlio.rviz preset: Fixed Frame camera_init, CloudRegistered/Path displays armed) accumulating geometry with path tracing lap. If map appears but smears during motion → round-two: de-skew normalization and/or real extrinsic.

Repo

GitHub (web-upload/drag-drop workflow): pandar40p-mobile-mapper. Latest: /mnt/user-data/outputs/pandar40p-mobile-mapper-v3.zip (48 files incl. 12 teardown photos): README (status current), docs/{build_guide,bench_test_checklist,pinout,t1_ethernet,zoox_quirks,teardown/}, hardware/bom.md, firmware/imu_bridge (v2 main.cpp w/ 0x27), ros2/imu_bridge_node (v2 demux: 0xAA 0x55 IMU 32B / 0x56 GPS var / 0x57 PPS 16B), launch/rig.launch.py, scripts/, UPDATE_NOTES_2026-07-26.md. PENDING: user uploads v3 + first-light/Bobo screenshots. Other outputs: imu_gps_bridge_v2.zip, rig_launch_v2.py, simulated_scan_preview.html, street_scan_endproduct.html.

After SLAM works

Real extrinsic_R from mount geometry (45° tilt + ICM orientation — user to describe board placement), live carried-lap test, then: stroller build → camera aim/bond/calibrate (Koide direct_visual_lidar_calibration + checkerboard intrinsics; allan_variance_ros overnight) → first walking capture → offline pipeline (GLIM humble CUDA binaries, HBA in Docker, ERASOR, PINGS/Gaussian-LIC2 splats; 8GB VRAM = chunk scenes). udev symlink for XIAO port = future nicety. Research tiers: Tier 1 = GLIM + IMU calib + correction files + Koide; Tier 2 = RTK F9P + HBA + dynamic removal + splats; skip tactical IMU & higher-res cameras.

User style: verify-everything-against-primary-sources doctrine (vindicated: Lemo pinout, BCM89811, ICM 0x27), hands-on welder/tinkerer, prefers web GitHub upload, dog named Bobo ("Bo").
