# Bill of Materials & Status

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | Hesai Pandar40P #1 | teardown donor | T1 fleet variant; BCM89811 confirmed; optics/parts salvage |
| 2 | Hesai Pandar40P #2 | **VERIFIED WORKING** | PA40-Zoox fw 2.20.17; factory-reset fix applied; rings in RViz 2026-07-26 |
| 3 | BUELEC 100/1000Base-T1-TX-E converter | **in service** | 100M + MASTER + orange pair = link; mounted on mast |
| 4 | Laptop i7-12650H / RTX 4060 8 GB / 32 GB, GbE | owned | Ubuntu 22.04 dual boot; capture + SLAM + post |
| 5 | ICM-42688-P breakout (Teyleten) | **in service** | wired to XIAO (SPI), 0x27 regs (DS-000347 verified), ~186 Hz, physics checked |
| 6 | MPU6050 | retired to drawer | ICM went straight in; never needed |
| 7 | XIAO ESP32-S3 | **in service** | v2 firmware (IMU+GPS+PPS capable); GPS wiring pending |
| 8 | WHEELTEC/FDI N100 IMU | candidate | 9-axis, 400 Hz raw, USB, ROS2 driver — would replace 5+7 |
| 9 | ELP-USB3DGS1200P01-H120 dual GS cameras **×2** | on rig (mockup) | in printed cases on ball mounts; aim→bond→calibrate pending; no external trigger pad (email ELP) |
| 10 | u-blox M10 GNSS | owned | position only; PPS/timing not required (PTP plan) |
| 11 | 18–20 V tool battery + adapter plate + 3 A fuse | owned | lidar power only; ~2.5 h per 5 Ah |
| 12 | Jogging stroller (pneumatic tires) | to acquire | air tires = vibration fix |
| 13 | Mast + brackets | to build | 1.8–2.0 m, triangulated, tilt holes 30/40/45° |
| 14 | Checkerboard target | owned | camera intrinsics |
| 15 | u-blox ZED-F9P RTK + L1/L2 antenna | Tier 2 | free InCORS NTRIP corrections (Indiana) |

Rejected/parked: NEMA+slip-ring rotation rig (motion replaces it), Hailo HAT
(RTX 4060 covers it), SBC compute (laptop wins), RAD-Moon 2 (cables not confirmed),
tactical IMU & higher-res cameras (poor quality/$ per research pass).
