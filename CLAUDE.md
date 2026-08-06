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
  - `~/bags/run_20260801_144508` (251 s, 2,514 lidar frames, 50,554 IMU,
    8.8 GB) — **the outdoor reference, and the only bag with a closed
    loop.** 234 m sidewalk loop on the stroller, returns to its start, so
    it is what drift is scored against: 0.55 %. See "First outdoor run"
    below. Map at `~/map_run_20260801_144508_level.pcd` (465 MB). GPS was
    disconnected for this run.
  - `~/bags/allan_20260801_031155` (8.58 h, 6.21 M IMU samples, 2.3 GB) —
    static overnight capture with the lidar unplugged, for the Allan
    variance. Contains no lidar data.
  - `~/bags/run_20260801_014240` (67 s, 678 lidar frames, 13,651 IMU) —
    **the first post-reseat bag and the indoor reference.** `bag_grav.py`
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
- **GPS** — u-blox M10 via XIAO, **TX→D5 at 9600 8N1, working as of
  2026-08-01**: `/gps/fix` publishes 1.0 Hz `NO_FIX` indoors, which is
  correct. PPS→D4 has never been wired and is not needed (see below).
  Verified after the rewire, alongside the IMU at 201.2 Hz:

  | topic | before | after |
  |---|---|---|
  | `/gps/fix` | 0 msgs | **1.0 Hz NO_FIX** |
  | `/gps/pps` | 670 Hz flood | **0 msgs** |
  - **That fully explains the "PPS flood".** D4 has been floating since
    day one, and the XIAO firmware raises an interrupt on it and forwards
    `0xAA 0x57` packets over serial, which the bridge republishes on
    `/gps/pps`. The 2026-08-01 outdoor bag caught 1,809 of them in a 2.6 s
    burst at ~50 kHz, then nothing for 226 s. No GPS emits that. The old
    "~840/s quirk without fix" was never GPS behaviour at all.
  - **Fixed and FLASHED 2026-08-01**: `firmware/imu_bridge/src/main.cpp`
    sets `PIN_PPS` to `INPUT_PULLDOWN` instead of plain `INPUT`. The
    u-blox TIMEPULSE output idles low and pulses high, so a pull-down is
    the correct termination whether or not the GPS is attached. Verified
    on the device afterwards: `/gps/pps` 0 msgs, `/gps/fix` 1.0 Hz NO_FIX,
    IMU 201.0 Hz with median 4.973 ms intervals and zero gaps.
  - **Flashing the XIAO**: PlatformIO is installed in user space
    (`~/.local/bin/pio`). From `firmware/imu_bridge/`:
    `pio run` builds (~48 s), `pio run -t upload` flashes (~12 s, auto-resets
    via RTS, no BOOT button needed). **Stop the bridge first** — it holds
    `/dev/ttyACM0` open and the upload will fail otherwise.
  - **PlatformIO needs `python3.10-venv`** (`sudo apt install -y
    python3.10-venv`). Without it, any operation that makes PlatformIO
    rebuild its virtualenv fails with *"ensurepip is not available"*.
    Installing `virtualenv` via pip does **not** help — this PlatformIO
    version calls `python3 -m venv` directly with no fallback.
  - **Firmware pin assignments, read from source rather than from this
    file** (`main.cpp:33-34`, `:115`): `PIN_GPS_RX = D5` at **9600 8N1**,
    `PIN_PPS = D4`. GPS TX goes to **D5**.
  - **There are TWO distinct PPS faults; do not conflate them.**
    - *Floating pin.* PPS has never been wired, so D4 sits unterminated
      with a rising-edge interrupt armed on it, and rings on electrical
      transients. This is the long-standing "~840/s without fix" and the
      2026-08-01 outdoor bag's 1,809 edges in 2.6 s at ~50 kHz. **The
      `INPUT_PULLDOWN` firmware change fixes this one.**
    - *NMEA driven onto D4.* On 2026-08-01, during the post-IMU-rewire
      reconnection only, GPS TX was landed on D4 instead of D5. That gives
      `/gps/fix` silent (the UART on D5 sees nothing) and `/gps/pps` at
      ~670 Hz (every serial edge trips the interrupt). **A pull-down would
      NOT have prevented this** — a UART actively drives the line, and a
      weak internal pull-down cannot suppress a driven signal. Wiring fix
      only. This fault was new that day; the wiring was correct before the
      2026-07-31 IMU rewire, so it explains nothing historical.
  - **PPS is not needed for next-step 7.** PTP syncs the lidar to the
    laptop over Ethernet; GPS is not involved. PPS only matters if the
    laptop clock is later disciplined to UTC (`gpsd` + `chrony`), which is
    Tier 2 / RTK territory, not SLAM.
- **RTK GNSS — Quectel LG290P, working 2026-08-06.** Bought instead of the
  ZED-F9P the research pass had planned; every doc that still said F9P was
  corrected the same day. Full detail: **docs/rtk_gnss.md**.
  - Identified from the device, not the box: `$PQTMVERNO` returns
    `LG290P03AANR01A06S`, firmware 2025/09/18. Rover mode
    (`PQTMCFGRCVRMODE,1`), 10 Hz (`FIXRATE,100`), **460800 8N1**
    (`CFGUART,1,460800,8,0,1,0`) over a QinHeng CH343 (`1a86:55d3`).
    Baud was scanned, not assumed — 460800 gave 216 checksum-valid NMEA
    sentences, every other rate gave zero.
  - **Six constellations, confirmed twice independently.**
    `$PQTMCFGCNST,OK,1,1,1,1,1,1` (all slots on), and the talker IDs on the
    wire: GP GPS, GL GLONASS, GA Galileo, GB BeiDou, GQ QZSS, GI NavIC.
  - **Mountpoint is `MSM4_VRS` and that is structural, not taste.** Legacy
    RTCM 3 observation messages exist only for GPS and GLONASS (1001-1004,
    1009-1012); there is no legacy type for Galileo or BeiDou. On an
    `RTCM3_*` mountpoint a four-constellation rover silently receives
    nothing for half its constellations. Measured over 75 s on MSM4_VRS:
    74 each of 1074/1084/1094/1124.
  - **Traffic is bidirectional and that is not optional.** Every InCORS
    mountpoint advertises `nmea=1`, and VRS synthesises observations at the
    position you report — send no GGA upstream and there is nothing to
    synthesise. `scripts/gnss/ntrip_rover.py` does both directions.
  - **Reached RTK float (GGA quality 5) in 1.8 s. Has never reached fixed
    (4).** Expected indoors. Float is visibly float: over 52 s on a
    stationary antenna with healthy corrections (age 1.1 s, 29 sats) the
    solution wandered 2.79 m in latitude and 2.71 m in altitude. **Only
    quality 4 is worth georeferencing with.** Outdoor test not yet done.
  - **InCORS is FREE** — confirmed by Matt 2026-08-06, who holds the
    account. The sourcetable advertises `fee=Y` on every mountpoint; that
    evidently just encodes "registration required". Do not re-raise it.
  - **Credentials are NOT in the repo** — `~/.config/ntrip/incors.conf`,
    mode 600. pandar-40p is public on GitHub. `.gitignore` excludes
    `incors.conf`; only a masked template is tracked. Caster address is
    masked in the repo too.
  - **What RTK does NOT do: FAST-LIO2 ignores GPS entirely.** It buys
    georeferencing and an independent, continuous drift score (today drift
    is only measurable when a run happens to close a loop). It does not
    improve the live trajectory.
  - Still open: antenna not mounted (must be the highest thing on the rig —
    the lidar is a spinning metal cylinder and will occlude it), lever arm
    to the lidar not measured, no ROS integration, not recorded into a bag.
    Note the u-blox ROS drivers are the **wrong** ones — this is Quectel,
    speaking NMEA + PQTM, so `nmea_navsat_driver` is the closer fit.
- **`/dev/ttyACM0` IS CONTESTED — new landmine 2026-08-06.** The LG290P
  enumerates as a CH343 USB-serial and claims `/dev/ttyACM0`, which is
  exactly what the IMU bridge uses. It took ACM0 with the XIAO unplugged.
  With both attached, **enumeration order decides who gets it**, and the
  loser silently talks to the wrong device — the bridge would parse NMEA as
  IMU packets, or RTCM would be written into the IMU. The old "udev symlink
  is a wanted nicety" note is upgraded to required.
  `scripts/gnss/99-rig-serial.rules` pins the GNSS by serial number
  (`5B90166916`, read from `udevadm`); the XIAO half is deliberately left
  blank because the board was not plugged in to read its IDs, and guessing
  them is exactly the habit this project keeps getting burned by.
- **Cameras** — 2× ELP-USB3DGS1200P01-H120 dual-lens global shutter
  (OG02B10, 3200×1200, USB2 UVC, MJPEG-always). Mounted, NOT aimed/bonded/
  calibrated yet. Calibrate LAST, after aim is final. **Only ONE lens per
  board is used** — the boards are dual-lens for cost, not for stereo, and
  the two in-use lenses are splayed ±30–35° for coverage. So camera↔camera
  sync is irrelevant; what matters is each camera's timestamp relative to
  the *lidar*.
  - Devices `/dev/video2` and `/dev/video4` (USB `32e4:2b10`). The odd
    nodes are metadata-only.
  - **Every mode is the combined side-by-side dual-lens frame** — 3200×1200
    is 2 × 1600×1200, 2560×720 is 2 × 1280×720, and so on down. There is
    no single-lens mode, so using one lens still pays full bandwidth for
    both and you crop in software.
  - **USB 2.0 bandwidth measured 2026-08-01.** Both cameras and the XIAO
    all sit on Bus 001, the USB 2.0 root hub, behind Corechips hubs; this
    laptop has a single xHCI controller so USB 2.0 devices cannot be
    spread across buses.

    | config | result |
    |---|---|
    | one camera alone, 3200×1200 | 53.4 fps |
    | **both, 3200×1200 @ 30** | **FAILS** — `VIDIOC_STREAMON: No space left on device` |
    | both, 3200×1200 @ 15 | works, measured 15.0 / 14.4+ fps |
    | both, 2560×720 @ 30 | works |

    That is isochronous bandwidth *reservation* exhaustion, not MJPEG data
    rate. **15 fps at full resolution is the usable ceiling for both**, and
    that is ample — at the measured 0.93 m/s walking pace it is 6 cm
    between frames.
  - **Camera streaming does NOT disturb IMU timing — tested 2026-08-01.**
    This was the one that could have blocked the panel bond.

    | | rate | median | p99.9 | max | gaps >15 ms |
    |---|---|---|---|---|---|
    | cameras idle | 201.6 Hz | 4.973 ms | 4.992 | 4.999 | 0 |
    | both streaming @ 3200×1200 15 fps | 201.5 Hz | 4.972 ms | 4.993 | 5.002 | 0 |

    Indistinguishable. With the XIAO on root port 4 and the cameras behind
    the hub on port 8 they do not contend. Safe to proceed with aiming and
    bonding.
  - **A USB 3.0 hub will NOT raise the 30 fps ceiling.** A USB3 hub
    contains a separate internal USB 2.0 hub, and USB 2.0 devices connect
    to *that*, sharing its single 480 Mb/s upstream exactly as now. The
    only real fixes are one camera per root port with no hub between, or
    living at 15 fps — which is ample.
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
- Rig launch: **`ros2 launch ~/pandar-40p/launch/rig.launch.py`** = hesai
  driver + its RViz + sensor bridge + lidar_temp + rig_status + optional
  `record:=true` bag. Run it from the **repo** path: as of 2026-08-06 every
  node it spawns comes from the repo, so there is one file under version
  control instead of a Desktop copy that silently drifts.
  `~/Desktop/rig_launch_v2.py` is kept as an identical copy for muscle
  memory.
- Recording protocol: hold dead still 3–5 s at start (gravity/bias init).
- **Thermal monitoring, added 2026-08-01.** Both temperatures now record
  into every bag: `/imu/temperature` at 1 Hz and `/lidar/temperature` at
  0.2 Hz. The IMU's reading was already being sent in every packet by the
  XIAO firmware and silently discarded by the bridge — it is free data, and
  it is the covariate that makes accelerometer bias drift correlatable
  rather than mysterious, which matters because turn-on bias is the known
  cause of the creeping ax yaw residual. The lidar's comes from the console
  API via `ros2/lidar_temp_node/`. First readings: IMU 27.1 °C, lidar
  38.6 °C — the lidar runs ~11 °C hotter than the IMU beside it.
  Conversion `degC = TEMP_DATA/132.48 + 25` is transcribed from the
  ICM-42688-P datasheet and flagged in source as not yet confirmed.
- **Replay/export/analyse is one command**:
  `scripts/diagnostics/run_test.sh [bag] [voxel]` — preflight, mount
  signature, mapper, PCD export, extrinsic analysis, frame-drop count.
- **Extra ROS nodes added 2026-08-01**, both launched by
  `rig.launch.py`: `ros2/lidar_temp_node/` polls the lidar console for its
  die temperature, and `ros2/rig_status_node/` serves all rig state as JSON
  on :8080. Both are read-only and use BEST_EFFORT subscriptions so they
  cannot add back-pressure to the mapping pipeline.
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
  convergence.
  - **A/B RUN 2026-08-01 — INCONCLUSIVE, and that is the finding.**
    Same bag, same everything, loop-closure gap: `true` 1.342 m (0.57 %),
    `false` 1.287 m (0.55 %). Pinning looks 55 mm better — but the *same*
    config gave 1.277 m on the earlier replay, so `true` alone spans
    65 mm across two runs (replay is not deterministic; scan counts came
    out 2508 / 2511 / 2485). The est-OFF result sits inside that spread.
    **The estimator's wandering does not measurably hurt loop closure.**
    Config left at `true`, which is what produced every known-good map.
    Settling it properly needs ~4 repeats of each for error bars, roughly
    35 min of replay, and the prize is a difference of ~0.02 % of path —
    not worth it unless something else makes the extrinsic suspect.
- ~~GPS went silent outdoors.~~ **Not a fault — the GPS was simply not
  reconnected after the 2026-07-31 IMU rewire.** `/gps/fix` recorded 0
  messages because nothing was attached. The run is fully valid; FAST-LIO2
  ignores GPS entirely.
  - **But it did reveal what the "PPS flood" actually is: a floating
    input, not a GPS quirk.** With nothing connected, `/gps/pps` produced
    1,809 messages in a **2.6 s burst at t = 22 s** and nothing across the
    other 226 s, at a median interval of 0.02 ms — **~50 kHz**. No GPS
    emits that. D4 is floating and rang on some electrical transient. The
    old note called this a "~840/s quirk without fix"; it is an unterminated
    pin. Enable an internal pull-down on D4, or just keep the GPS
    connected, and it should go away.

## The "doorway error" was probably never an error (2026-08-01)

The long-standing "0.77 m vs 0.813 m nominal" residual looks like a wrong
nominal rather than a mapping fault. Three things say so:

- **Scale is fine.** Floor→ceiling reads 3.0445 ± 0.0035 m against
  3.0607 m taped (10 ft 0½ in): **−0.53 % over 3 m** on the post-QoS-fix
  map, and −1.92 % even on the pre-fix one. A map genuinely 12 % small
  would have put that ceiling at 2.69 m. Errors that are 0.5 % on one
  baseline and 12 % on another are not scale errors — scale is
  proportional by definition.
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
   verified against the ceiling to −0.53 % over 3 m post-QoS-fix (−1.92 %
   before it); the doorway shortfall
   is very likely a 28-inch door being compared to a 32-inch nominal. See
   "The 'doorway error' was probably never an error" above. Residual task
   is a tape measure on the real doorway, low priority.
3. ~~Confirm or kill the frame-drop suspicion.~~ **DONE 2026-08-01 — it
   was real and it is fixed.** BEST_EFFORT depth 5 was dropping two-thirds
   of the cloud; reliable QoS takes it to 99.4%. See the landmine entry.
   The 8/01 bag was re-mapped through the fix the same day and metric
   accuracy improved with it: floor→ceiling went −1.92 % → **−0.53 %**, a
   +42.7 mm change against ±3.5 mm of method noise. Recovering the frames
   did not just add points, it made the map measurably more accurate.
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
   **Nothing blocks the bond, measured 2026-08-01.** Both cameras sustain
   3200×1200 simultaneously at 15 fps (30 fps fails on USB 2.0
   isochronous bandwidth), and streaming them has **no measurable effect
   on IMU timing** — 201.5 vs 201.6 Hz, identical intervals, zero gaps.
   See the Cameras entry under Hardware. Do intrinsics *after* bonding,
   and set 15 fps before calibrating so you calibrate at the rate you
   will record at.
6. ~~Stroller acquisition + mast-to-stroller build.~~ **DONE 2026-08-01**,
   confirmed by Matt. The platform is a **jogger stroller chosen for its
   pneumatic tires**, and that choice is now validated by measurement: small hard wheels
   transmit sharp shock over sidewalk joints, and the accelerometer has
   only 6.7× headroom to its ±8 g full scale against the 11.74 m/s² peak
   of an indoor hand-carried pass. Clipping the accel corrupts
   integration silently. Air tires are the cheapest fix; compliance
   between the chassis and the sensor head is the next one — never
   between the IMU and the lidar, which must stay rigid.
   **Rubber isolator mounts: NOT NEEDED, settled by measurement
   2026-08-01.** The 234 m sidewalk run peaked at 32.59 m/s^2 — 42 % of the
   ±8 g full scale — with zero samples above even 50 %. Pneumatic tires
   did the job. Had it been marginal, the rule was: size isolators against
   a measured shock spectrum, never buy blind, because a mount whose
   natural frequency sits inside the disturbance band amplifies rather than
   damps, and rubber loaded far below its rating stays effectively rigid.
7. PTP time sync; revert `use_timestamp_type` to 0. **Do it in this exact
   order, and do NOT leave ClockSource on GPS.** The console's clock
   source select is `0 = GPS, 1 = PTP` and it currently reads **0**. That
   is harmless today only because `use_timestamp_type: 1` makes the driver
   use host receive time and ignore the sensor clock entirely. The moment
   you switch to type 0:
   - `ClockSource = GPS` **with a fix** puts lidar stamps in GPS/UTC while
     the IMU stays on host time — a timestamp-domain split, which is
     blocker (3) from the debugging history all over again, and it will
     fail silently.
   - `ClockSource = GPS` **without** a fix free-runs from the Y2K epoch,
     which is the same trap from the other direction.
   So: set `ClockSource` → **PTP (1)**, run `ptp4l` as master on the
   laptop, confirm `PTPStatus` leaves "Free Run", *then* set
   `use_timestamp_type: 0`. Verify by echoing header stamps, not with
   `ros2 bag info`, whose Start/End come from the recorder's wall clock.
   **This machine has NO PTP hardware, every route checked 2026-08-01.**
   `ethtool -T` reports `PTP Hardware Clock: none` on all of:

   | interface | part | speed |
   |---|---|---|
   | `enp4s0` | r8169 onboard | 100 Mb/s linked |
   | `wlp3s0` | mt7921e wifi | — |
   | `enx00e04c68102f` | USB `0bda:8153` RTL8153 | gigabit |
   | `enx803f5dd0c645` | USB `0bda:8156` RTL8156 | 2.5 GbE |

   Both USB adapters run the `r8152` driver, which implements no PTP for
   any part in that family, and both report only `software-receive` —
   *weaker* than the onboard NIC, which at least offers
   `software-transmit`. Using either would be a downgrade. Do not re-test
   these; each was identified individually with the other unplugged. Note
   also that 2.5 GbE buys nothing on the lidar path regardless: the BUELEC
   converter caps it at 100 Mb (`rate=100M`, and `enp4s0` links at exactly
   100 Mb/s), against the lidar's ~37.6 MB/s. So any PTP here is
   software-timestamped, tens of microseconds at best.
   **What it is worth, at the measured 0.93 m/s walking pace:**

   | timing source | position error |
   |---|---|
   | host receive time (current) | 0.465 mm |
   | software PTP (achievable) | 0.047 mm |
   | hardware PTP | not available |

   **The entire upgrade buys ~0.4 mm against 1,277 mm of measured drift.**
   It is correctness, not accuracy, and it carries a real risk: type 0
   without a solid PTP lock is the silent timestamp-domain failure that
   cost days in July. **Recommendation: defer** until something makes
   timing matter — the rig on a vehicle, or cameras with hardware sync.
   The `ptp4l` master config is ready at `ros2/config/ptp4l_lidar.conf`
   when that day comes.
8. ~~Longer outdoor capture with a closed loop to quantify drift.~~
   **DONE 2026-08-01** — `run_20260801_144508`, a 234 m sidewalk loop
   returning to its start. **0.55 % drift**, of which only 10 cm is
   vertical. See "First outdoor run" above. Frame accounting held at
   99.8 % under 4× the indoor data volume. Further captures are now about
   coverage rather than about establishing the number.
9. Offline chain: GLIM (humble CUDA binaries) → HBA (Docker) → ERASOR
   dynamic removal → colorize → PINGS/Gaussian-LIC2 splats. 8 GB VRAM ⇒
   chunk scenes. Tier 2: **RTK is no longer 'later' — the LG290P works as of
   2026-08-06** on free Indiana InCORS NTRIP. See the RTK entry under Hardware
   and docs/rtk_gnss.md. Remaining: antenna mount, RTK fixed outdoors, ROS
   integration, and recording it into a bag.

## Decisions taken 2026-08-01/02 that are not visible in the code

Recorded because they are cheap to re-litigate and the reasoning is not
obvious from any config file.

- **Spin rate stays 600 rpm.** 1200 rpm does not add data — the firing
  rate is fixed, so it halves azimuth resolution (0.2° → 0.4°) to double
  the frame rate. FAST-LIO2 already deskews with per-point timestamps, so
  the usual motion-blur argument for 20 Hz does not apply at walking pace,
  where a 100 ms scan sweeps ~14 cm. 1200 rpm earns its keep at vehicle
  speeds, not on a stroller.
- **Dual return stays on.** Matt's call, for outdoor work. Confirmed
  worthwhile by the 2026-08-01 map, where tree canopy shows internal
  structure rather than a solid shell — second returns punching through
  foliage. Indoors it is mostly edge noise at double the bandwidth.
- **Bags stay on the root NVMe; the second M.2 is not used.** Recording
  demands 37.6 MB/s against a drive doing 1000+, and the recorder captures
  678 frames in 67.7 s = 10.02 Hz, i.e. **100 %** of what the sensor
  emits. Disk was never a bottleneck — the losses were all QoS. Capacity
  is the only real argument: 37.6 MB/s is **135 GB/hour**, so ~6.4 hours
  of continuous recording fits in the free space. The second drive is a
  single 954 GB NTFS partition labelled "Storage", unmounted, contents
  unknown; NTFS via ntfs-3g is FUSE-based and poor for sustained writes,
  so it would want reformatting before use.
- **Battery monitoring — BUILT AND FLASHED 2026-08-01, awaiting wiring.**
  `firmware/battery_monitor/`. INA226 (0–36 V, I2C, 2 mΩ shunt) plus a
  Waveshare ESP32-C6-LCD-1.47, watching the **flooded lead-acid deep-cycle**
  pack. Wiring, confirmed against the board pinout diagram: INA226 VCC →
  3V3(OUT) and GND → GND on the left header, SDA → GP18 and SCL → GP19 on
  the right header, shunt high-side in the battery positive lead.
  Thresholds are the deep-cycle ones that protect cycle life: amber at
  **12.2 V = 50 % depth of discharge**, red at 12.0 V — not the 11.8 V
  "flat" figure, since habitually going past 50 % is what kills these
  packs. All resting voltages; the display flags when current is flowing,
  because a loaded pack sags and reads low. Design notes: the INA226
  runs at 3.3 V so it wires straight to the ESP32 with no level shifting;
  its 0–36 V common-mode range means **high-side** sensing works, which
  keeps one common ground across laptop, lidar and converter. **Fuse at
  the battery terminal** — a deep cycle will push hundreds of amps into a
  short and this is new wiring on a moving cart. Thresholds depend on
  chemistry (12.0 V is ~50 % on flooded lead-acid but near-full on
  LiFePO4), so establish that first. The payoff worth building for: publish
  it as a ROS topic and record it in the bag, so a bad run can be checked
  against a voltage trace instead of guessed at. Watch for a pin clash —
  the XIAO's GPS sits on D5/D4, which are the ESP32-S3's default I2C pins;
  remapping to D2/D3 should work but needs checking against the schematic.
  - **A hardware low-voltage ALERT is wired to GP20** and drives the RGB
    LED, because the point is to be noticed while pushing the stroller and
    not looking at a screen. The INA226 raises it itself, so a hung sketch
    cannot swallow it. Threshold 12.0 V rather than the 12.2 V half-charge
    line: a pack sags ~0.1 V under a few amps, so alerting at 12.2 while
    recording would cry wolf.
  - **Two C6-specific build traps**, both cost a failed build: Arduino_GFX
    1.4.x namespaces colours as `RGB565_*`, and `ARDUINO_USB_CDC_ON_BOOT=1`
    needs `ARDUINO_USB_MODE=1` alongside it, because the C6 has USB
    Serial/JTAG rather than the native USB-OTG that `USBSerial` requires.
    The stock espressif32 platform also pins Arduino to 2.0.17, which
    predates the C6 — hence the pioarduino fork in `platformio.ini`.
  - **Every screen carries the build date and time.** Without it a board
    showing the same "NO INA226" message before and after a reflash gives
    no way to tell which firmware is running.
- **A display on the XIAO would be for stillness and clipping, not tilt.**
  Starting attitude does not need to be consistent — the levelling rotation
  is derived per-run from logged gravity. The readouts actually worth
  having are live |gyro| (so the 3–5 s dead-still init is verified rather
  than counted) and live peak |accel| against the ±8 g limit, since
  clipping is silent. An I2C display needs 2 pins; the round display wants
  SPI plus CS/DC/backlight and would not fit alongside the IMU and GPS.
- **The rig dashboard is laptop-fed over WiFi, not sensor-fed.**
  `ros2/rig_status_node/` aggregates everything the laptop already knows —
  sensor rates and staleness, both temperatures, GPS fix and position,
  peak accel as a percentage of the ±8 g full scale, still-detection for
  the dead-still init, disk headroom converted to recording hours at the
  measured 135 GB/h, and whether a bag is recording — and serves it as
  JSON on :8080. A display then only renders; adding a new reading later
  is a laptop-side change rather than a reflash. It is already useful with
  no new hardware, since it is reachable over Tailscale from a phone.
  The M5Stack Tab5 is the intended screen (ESP32-P4 with an
  ESP32-C6-MINI-1U co-processor, so it has WiFi 6 without extra parts;
  5" 1280×720 touch; NP-F550 battery, ~6 h). The C6 monitor keeps a
  distinct job: it works with the laptop **off**, which no WiFi dashboard
  can. Temperatures are reported in both °F and °C in the JSON, but the
  **ROS topics stay Celsius** because `sensor_msgs/Temperature` is defined
  that way and changing it would break the contract silently.
- **Lidar stays primary for pose; cameras become primary for the model.**
  Matt raised whether cameras should lead instead. For *trajectory*, lidar
  ranges directly while stereo error grows with the square of distance,
  and blank drywall and direct sun defeat feature matching — trajectory
  error poisons everything downstream, so that half stays lidar-inertial.
  For the *deliverable*, cameras carry appearance and fine geometry and
  lidar supplies scale and drift control. That is exactly what
  Gaussian-LIC2 and PINGS already do, both of which are on the offline
  chain in next-step 9. The roadmap is not "lidar map, then paint it".

## Where the raw evidence lives

Claims in this file are traceable to data still on disk. If a number here
looks wrong, re-derive it rather than trusting the prose.

- **Bags** — `~/bags/`, listed at the top of this file. 13.5 GB total,
  862 GB free, no pressure to prune.
- **Maps** — `~/map_run_<stamp>.pcd` and `_level.pcd`. Also
  `map_run_20260801_014240_PREQOS.pcd`, deliberately kept: it is the same
  bag mapped through the old dropping QoS, and is what the −1.92 % vs
  −0.53 % scale comparison rests on.
- **Extrinsic logs** — `~/ros2_ws/src/FAST_LIO/Log/`:
  `mat_out.txt` is transient and gets overwritten by every replay, so the
  ones that back specific claims are kept under names:
  `mat_out_20260730bag_crooked.txt` (the 14° crooked-mount forensic run),
  `mat_20260801outdoor_eston.txt` and `mat_20260801outdoor_estoff.txt`
  (the `extrinsic_est_en` A/B).
- **RTK GNSS** — `scripts/gnss/`: `gnss_probe.py` finds the port and baud by
  scanning, `gnss_query.py` asks the receiver what it is (read-only PQTM),
  `ntrip_rover.py` streams corrections and reports fix quality. Credentials
  are outside the repo at `~/.config/ntrip/incors.conf`.
- **Re-deriving anything** — `scripts/diagnostics/` covers it:
  `bag_grav.py` mount signature, `analyze_ext.py` extrinsic,
  `allan.py` IMU noise, `floorplan.py` levelling and map measurement,
  `lidar_config.py` console settings, `run_test.sh` the whole replay.

**Replay is not deterministic.** Three replays of the same bag gave 2508 /
2511 / 2485 processed scans and loop-closure gaps spanning 65 mm. Any A/B
resting on a difference smaller than about 100 mm of loop closure needs
repeats before it means anything.

## Working style with Matt

- Verify against primary sources before asserting; prefer measurements over
  inference and say which one a claim rests on. When a hypothesis is ruled
  out by data, retract it explicitly.
- One concrete next step at a time when debugging; small standalone scripts
  over CLI incantations; paste-ready commands with absolute paths.
- Diff proposed doc changes against the live repo rather than assuming a
  rewrite wins. Matt handles image links himself.
- **Record decisions that came from discussion, not just from
  measurement.** Observed failure 2026-08-01: every experimental result
  was committed within minutes, while four decisions reached purely by
  reasoning — spin rate, the second SSD, the battery-monitor design, the
  camera-vs-lidar primacy question — existed only in chat until an audit
  caught them. Running an experiment and writing it up feels like one
  action; answering a question does not. If a question got settled with a
  reason, that reason belongs here, or it will be re-litigated.
- Push straight to main (standing permission, 2026-07-31) — no PRs or
  side branches unless Matt asks.
- The dog is Bobo. He has appeared in both the first camera image and the
  first point cloud, and has a standing role as scan-quality control.
