# Mobile LiDAR Mapping Rig — Build Guide v2
### Hesai Pandar40P · Stroller Platform · Laptop SLAM · Maximum-Quality Colorized Maps
*v2 integrates the deep-research findings (Jul 2026). Supersedes v1.*

**Goal:** Walk the house or down the middle of the street pushing a modified jogging
stroller and capture the densest, most globally-consistent, colorized 3D map this
hardware can produce — real-time SLAM on the laptop while walking, heavyweight
refinement and photoreal outputs (meshes, Gaussian splats) afterward on the RTX 4060.

**Two doctrines that now govern everything (from the research pass):**
1. **Record raw, always.** Every bag captures raw lidar packets, IMU, camera frames,
   and GNSS — even when live SLAM is running. Nearly every quality upgrade is
   software applied to recorded data; disciplined capture makes every future
   pipeline improvement retroactive.
2. **Real-time is for confidence; offline is for quality.** The live FAST-LIO2 map
   tells you coverage is good while walking. The *deliverable* map comes from the
   offline chain (GLIM global optimization → HBA/BALM refinement → cleanup →
   colorization/splats).

---

## 1. System Architecture

```
                    STROLLER MAST HEAD (~2 m, rigid, braced)
   ┌──────────────────────────────────────────────────────┐
   │  Pandar40P  (tilted 45° forward, spin axis fwd/up)   │
   │  ICM-42688-P IMU ──SPI── XIAO ESP32-S3 bridge        │
   │  2× ELP dual-lens GS camera boards (splayed ±30°,    │
   │     pitched up ~10–15°, in the lidar's blind cone)   │
   │  GNSS antenna (M10 now → ZED-F9P RTK in Tier 2)      │
   └──────────────┬───────────────────────────────────────┘
                  │  Ethernet (lidar) + USB (IMU bridge,
                  │  cameras, GNSS)
                  ▼
        LAPTOP in stroller seat (lid open = live RViz)
        i7-12650H · RTX 4060 8GB · 32 GB · Ubuntu 22.04 · ROS 2 Humble
        LIVE:    hesai_ros_driver → FAST-LIO2 (confidence view)
                 rosbag2 recording ALL raw topics to NVMe
        OFFLINE: GLIM (global optimization) → HBA/BALM refinement
                 → dynamic removal → colorization → mesh / 3DGS
                  ▲
        12 V-class tool battery → lidar ONLY (~18 W, fused)
```

**Time sync:** laptop = single clock master. Lidar via **PTP** (`ptp4l` master on
the wired NIC; lidar clock source set to PTP in web control — works indoors and
out). IMU hardware-timestamped on the ESP32-S3 and correlated to laptop time.
Cameras software-timestamped on arrival. GNSS = position, not timing.

---

## 2. Bill of Materials & Status

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | Hesai Pandar40P | Purchased ($149, 30-day warranty from Jun 27 2026) | 40 ch, −25°/+15°, ~200 m, DC 9–48 V ~18 W max 3 A, no power switch, lens film must come off |
| 2 | Laptop (i7-12650H, RTX 4060 8 GB, 32 GB, GbE) | Owned | Capture + SLAM + all post-processing. 8 GB VRAM is the known ceiling for big splat scenes — chunk large scans |
| 3 | ICM-42688-P breakout | Ordered | Primary IMU. Verify "42688" marking on arrival. **Allan-variance calibrate before first mapping run (Tier 1)** |
| 4 | MPU6050 | Owned | Bring-up stand-in only |
| 5 | XIAO ESP32-S3 | Owned | USB IMU timestamping bridge, 200+ Hz |
| 6 | 2× ELP dual-lens GS boards (OG02B10) | To order | **Confirm COLOR in writing.** One lens used per board. Research verdict: do NOT chase higher-res cameras — poor quality/$ here |
| 7 | u-blox M10 | Owned | Tier-1 GNSS (meter-level). Superseded outdoors by #13 |
| 8 | 16-pin waterproof connector pair | Ordered | Replaces the Lemo mid-cable |
| 9 | Jogging stroller, pneumatic tires | To acquire | Air tires = the vibration fix; keep lidar+IMU rigidly coupled as one head (never isolate them from each other) |
| 10 | Mast + brackets | To build | ~2 m, triangulated bracing, adjustable tilt (30/40/45°) |
| 11 | 18–20 V tool battery + plate + 3 A fuse | Owned/cheap | Lidar only; ~2.5 h per 5 Ah pack |
| 12 | Checkerboard target | Owned | Camera intrinsics (extrinsics now go targetless — see Tier 1) |
| 13 | **u-blox ZED-F9P RTK board + L1/L2 antenna** (SparkFun/Ardusimple, ~$200–275) | Tier 2 purchase | Dual-freq RTK. Corrections FREE via Indiana **InCORS** NTRIP (state CORS network covers Rushville area). Transforms outdoor georeferencing to cm-class |

---

## 3. Quality Tiers (research-integrated roadmap)

### Tier 1 — adopt before/at first light (cheap or free, high impact)
| Change | What/why |
|---|---|
| **GLIM added to the stack** | Global-optimization SLAM (GPU-capable, ROS 2 friendly). FAST-LIO2 stays as the live view; GLIM re-processes the same bags offline for globally consistent maps. Highest quality-per-effort finding of the research |
| **Record-raw doctrine** | rosbag2 captures raw lidar packets + IMU + camera + GNSS on every run, no exceptions |
| **IMU intrinsic calibration** | Overnight static log → Allan variance (allan_variance_ros) → real noise/bias params into FAST-LIO2/GLIM configs. Research verdict: proper calibration of the ICM-42688-P closes most of the gap to tactical IMUs → **tactical IMU purchase skipped** |
| **Targetless camera↔lidar extrinsics** | Koide's `direct_visual_lidar_calibration` — better extrinsics than checkerboard-only, no target ceremony. Checkerboard still used for camera *intrinsics* |
| **Lidar settings locked** | **600 rpm (10 Hz)** for max per-frame density at walking speed; single-return (strongest) initially; dual-return reserved as an experiment for vegetation-heavy streets. Verify no per-channel calibration anomalies against the auto-imported correction file |
| **Fixed exposure while scanning** | Lock camera exposure/gain per run (auto-exposure flicker degrades colorization consistency); pick per-environment |

### Tier 2 — after the rig works end-to-end
| Change | What/why |
|---|---|
| **RTK GNSS: ZED-F9P + InCORS NTRIP** | Free state correction network + ~$250 hardware = cm-level outdoor trajectory anchoring and true georeferencing. Fuse via GNSS factors (LIO_SAM_6AXIS / GLIM GNSS constraints). The standout big-ticket item |
| **Offline refinement chain** | GLIM output → **HBA** (hierarchical bundle adjustment) / BALM pass → measurably tighter walls/edges. Some tools are ROS 1 era — run them in Docker containers rather than fighting Humble |
| **Dynamic object removal** | Street scans: Removert / ERASOR / dynablox / BeautyMap-class tools to erase pedestrians+cars from the final map (RTX 4060 helps here) |
| **Splat pipeline** | LiDAR-seeded 3D Gaussian splatting (PINGS / Gaussian-LIC2-class pipelines): initialize splats from the refined cloud + camera frames + trajectory. Chunk scenes to respect 8 GB VRAM |
| **Intensity calibration + principled filtering** | Normalize Pandar40P reflectivity across range/angle; SOR/ROR outlier passes tuned to 40P noise character |

### Tier 3 — experimental (clearly speculative, try when the rig is proven)
- **FAST-LIVO2**: lidar-inertial-**visual** odometry — the ELP cameras improving
  *geometry*, not just color. Unproven on this exact stack; high ceiling.
- **Multi-session merging**: repeated scans of the same house/street fused
  (MS-Mapping-class) for super-resolution-like density and change tracking.
- **Stereo-from-dual-lens**: the unused second lens per ELP board as a third
  geometry source in texture-poor zones. Almost certainly marginal; free to try.
- **Neural implicit surface (SDF) reconstruction** from lidar+camera on the 4060
  as an alternative to Poisson meshing.

### Explicitly skipped (research verdict: poor quality-per-dollar for this rig)
- Tactical/industrial IMU (ADIS/Epson/Murata class) — calibrated ICM-42688-P suffices at walking speed.
- Higher-resolution cameras — 2 MP GS already oversamples the lidar's angular density; splat gains don't justify cost/bandwidth.
- External rotation rig — motion replaces it (unchanged from v1).

---

## 4. Build Phases

### Phase 1 — Bench-test the lidar BEFORE modifying anything  ⏰ warranty
Follow **`pandar40p_bench_test_checklist.md`** verbatim. Critical path: test-hook
temporary hookup (pins 11+12 V+, 13+14 GND, 5/6 + 7/8 Ethernet) → laptop NIC
192.168.1.100/24 → lens film off → power (it spins instantly, no switch) → UDP
on port 2368 → web control at http://192.168.1.201 (save Device Log JSON) →
**PandarView 2** (Windows OK): full 360°, 40 channels on a flat wall, record
baseline PCAP. Gate: all healthy → proceed; anything wrong → warranty return.

### Phase 2 — Connector swap
Cut mid-cable (≥10 cm stub on the Lemo). Continuity-beep pin→color; the beep
outranks the manual table. Splice rules: power pairs doubled, Ethernet pairs
twisted to the shell on adjacent pins, pairs away from power, solder + adhesive
heat shrink; GPS wires carried on spares (pin 9 is RS232-level — never TTL).
Repeat the full bench test + wiggle test; `ethtool` = 100 Mb/s Full, zero RX errors.

**Pinout (manual §2.2.1):** 5/6 Eth RX−/+ (Blue/Blue-Wh) · 7/8 Eth TX−/+
(Orange/Orange-Wh) · 9 GPS NMEA RS232 (White) · 10 PPS (Yellow) · 11/12 Power
(Red/Gray) · 13/14 GND (Black/Gray-Wh) · 1–4, 15–16 unused.

### Phase 3 — Mechanical build
Jogging stroller, air tires. Mast to **1.8–2.0 m**, triangulated braces — if the
top wobbles by hand, SLAM sees fake motion. Lidar tilted **45° forward**
(alternate holes at 30/40°): fan sweeps ≈ −70°…+60° world elevation, rear beams
pass over the pusher's head (verify once in live view). IMU rigid on the head,
X-axis aligned to the lidar's X (**the 40P's X is at the 90° housing position,
not the cable exit**). Camera pod on the spin-axis side (inside the lidar's
blind cone → zero occlusion), boards splayed ±30°, pitched up 10–15°, rigid
brackets only. GNSS antenna topmost. If vibration ever proves problematic:
isolate the **entire head as one unit** (wire-rope/silicone between mast and
head) — never between lidar and IMU.

### Phase 4 — Electronics
Battery → 3 A fuse → lidar direct (9–48 V absorbs the discharge curve). Laptop
self-powered; cameras/GNSS/ESP32 on laptop USB (powered hub if short). Ethernet
is transformer-isolated — no ground-loop concern. IMU bridge: ICM-42688-P over
SPI at 200+ Hz, µs-stamped on the ESP32-S3, binary over USB-CDC; laptop node
publishes `sensor_msgs/Imu` with offset+drift clock mapping. Bring up MPU6050
first, swap to ICM with zero laptop-side changes.

### Phase 5 — Software install (Ubuntu 22.04)
1. Dual-boot Ubuntu 22.04 + NVIDIA driver. (Windows keeps PandarView 2 and
   remains fine for post-processing; capture/SLAM is Linux-only territory —
   the stack's packages, ptp4l, and raw UDP handling don't port.)
2. ROS 2 Humble desktop-full.
3. **HesaiLidar_ROS_2.0** (official; verified Pandar40P + Humble):
   clone --recurse-submodules → colcon build → config: source_type real-time,
   device_ip_address 192.168.1.201 → `ros2 launch hesai_ros_driver start.py`.
4. **FAST-LIO2** (live view). Known task: point-format config for the Hesai
   per-point timestamp + ring fields (community configs exist; one evening).
5. **GLIM** (offline quality path) — build with GPU support for the 4060.
6. **LIO_SAM_6AXIS** (outdoor GNSS fusion; stock LIO-SAM needs 9-axis — skip upstream).
7. Support: linuxptp, gpsd, chrony, v4l-utils, OpenCV, Open3D, CloudCompare,
   Docker (for ROS 1-era refinement tools), rosbag2.
8. Calibration tools: allan_variance_ros, Koide direct_visual_lidar_calibration.

### Phase 6 — Time sync
`ptp4l` master on the wired NIC → lidar Clock Source = **PTP** in web control →
confirm lidar stamps track ROS time. gpsd+chrony discipline the laptop from
GNSS outdoors. One clock domain everywhere.

### Phase 7 — Calibration (order matters)
1. **IMU intrinsics:** overnight static log → Allan variance → noise/bias into
   all SLAM configs. (Tier-1 quality lever; do before first mapping run.)
2. **IMU↔lidar extrinsic:** hand-measure translation; rotation from mount
   geometry (45°!); FAST-LIO2/GLIM online refinement polishes it.
3. **Camera intrinsics:** checkerboard per used lens.
4. **Camera↔lidar extrinsic:** Koide targetless direct visual-lidar alignment;
   redo any time a bracket is touched.
5. Validate: hand-rotate head vs `/imu/data_raw`; projected color lands on edges.

### Phase 8 — Capture workflow
Power lidar → launch driver + IMU bridge + FAST-LIO2 + **rosbag2 recording all
raw topics** → RViz sanity → walk. Technique: smooth pace; slow through doorways
and feature-poor zones; **pirouette each room** (one slow 360° sweeps the tilted
fan floor-to-past-vertical); **close loops** — revisit and end where you began;
gentle over curbs; fixed camera exposure per run; outdoor runs wait for GNSS fix.
Dwell time adds nothing — viewpoints add everything.

### Phase 9 — The quality pipeline (offline, RTX 4060)
1. Re-run bags through **GLIM** → globally optimized map + trajectory.
2. **HBA/BALM** refinement pass (Docker where ROS 1 tooling demands).
3. **Dynamic removal** on street scans (Removert/ERASOR/dynablox class).
4. Outlier filtering + intensity normalization.
5. **Colorize** from camera frames via calibrated extrinsics + refined trajectory
   (one lens per board in the pipeline).
6. Outputs: meshes (Poisson or neural-SDF experiment) and **lidar-seeded 3DGS**
   splats (chunked for 8 GB VRAM). Archive raw bags forever — captures are
   unrepeatable; pipelines improve.

---

## 5. Verified-facts appendix
| Fact | Source |
|---|---|
| Pandar40P: 40 ch, −25°/+15°, ~200 m; DC 9–48 V, ~18 W, label max 3 A | Hesai manual + unit label |
| No power switch; streams on power+link; lens film must be removed | Manual |
| Default IP 192.168.1.201; host 192.168.1.100/24; UDP 2368 (cloud, broadcast default), 10110 (GPS) | Manual |
| Web control http://192.168.1.201; spin 600/1200 rpm; Clock Source GPS/**PTP**; azimuth FOV masking exists | Manual/web control |
| GPS NMEA pin is RS232-level; PPS ≥1 ms; power off before GPS wiring | Manual |
| Lemo FGG.2T.316 ↔ PHG.2T.316 (T-series ≠ B-series); full pinout §2.2.1 | Manual |
| 40P auto-imports its calibration file; over-temp self-shutdown (flag, 60 s) | Manual |
| HesaiLidar_ROS_2.0 supports Pandar40P on 22.04/Humble | Official GitHub |
| Stock LIO-SAM requires 9-axis IMU; LIO_SAM_6AXIS adds 6-axis + low-cost GNSS | Official GitHubs |
| GLIM: global-optimization SLAM, GPU-capable, ROS 2 friendly — top research pick | Research report |
| Calibrated ICM-42688-P ≈ closes most gap to tactical IMUs at walking speed | Research report |
| ZED-F9P + free Indiana InCORS NTRIP = cm-class RTK for ~$200–275 hardware | Research report |
| Higher-res cameras & tactical IMU: poor quality/$ for this rig — skipped | Research report |
| 8 GB VRAM = chunk large splat scenes; some refinement tools need ROS 1/Docker | Research report |
| ELP dual-lens GS: OG02B10, 3200×1200 synced, USB2 UVC, 75–170° lens options | ELP pages |

## 6. Open items / verify-on-arrival
- [ ] Lidar bench test inside warranty window — **top priority**
- [ ] ICM board: "42688" marking, regulator range, CS continuity
- [ ] ELP cameras: written **color** confirmation before ordering
- [ ] Cable wire colors vs pinout (continuity beep at cut)
- [ ] FAST-LIO2 Hesai point-format config (one evening)
- [ ] GLIM build with CUDA on 22.04 (verify current repo instructions at build time)
- [ ] InCORS NTRIP account/mountpoint details for the Rushville area (Tier 2)
- [ ] Laptop lid-close + sustained-load thermal behavior in the stroller

## 7. Rejected ideas (and why)
External rotation rig (motion replaces it — parts reserved for a future
stationary cave-style spinner) · GPS-wired lidar timing (PTP does it everywhere,
no RS232 shifting) · Hailo HAT (RTX 4060 covers the AI use case) · SBC compute
(owned laptop wins every axis) · Windows/WSL2 capture (stack is Ubuntu-native;
PTP + raw UDP + USB passthrough all fight you) · Tactical IMU & high-res
cameras (research: poor quality/$) · Matterport (different tool: turnkey
interior visualization vs. this capture instrument; splats close the visual gap).
