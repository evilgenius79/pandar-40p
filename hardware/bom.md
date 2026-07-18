# Bill of Materials & Status

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | Hesai Pandar40P #1 | teardown donor | T1 fleet variant; BCM89811 confirmed; optics/parts salvage |
| 2 | Hesai Pandar40P #2 | arriving | expect T1 variant; DO NOT cut Lemo until streaming |
| 3 | BUELEC 100/1000Base-T1-TX-E converter | ordered | Marvell 88Q2112; terminal block + MATEnet/H-MTD adapters incl. |
| 4 | Laptop i7-12650H / RTX 4060 8 GB / 32 GB, GbE | owned | Ubuntu 22.04 dual boot; capture + SLAM + post |
| 5 | ICM-42688-P breakout (Teyleten) | ordered | verify "42688" marking, CS continuity on arrival |
| 6 | MPU6050 | owned | bring-up stand-in only |
| 7 | XIAO ESP32-S3 | owned | IMU timestamping bridge (see firmware/) |
| 8 | WHEELTEC/FDI N100 IMU | candidate | 9-axis, 400 Hz raw, USB, ROS2 driver — would replace 5+7 |
| 9 | ELP-USB3DGS1200P01-H120 dual GS camera | ordered | color OG02B10; one lens/board; 170° M12 lenses = later option |
| 10 | u-blox M10 GNSS | owned | position only; PPS/timing not required (PTP plan) |
| 11 | 18–20 V tool battery + adapter plate + 3 A fuse | owned | lidar power only; ~2.5 h per 5 Ah |
| 12 | Jogging stroller (pneumatic tires) | to acquire | air tires = vibration fix |
| 13 | Mast + brackets | to build | 1.8–2.0 m, triangulated, tilt holes 30/40/45° |
| 14 | Checkerboard target | owned | camera intrinsics |
| 15 | u-blox ZED-F9P RTK + L1/L2 antenna | Tier 2 | free InCORS NTRIP corrections (Indiana) |

Rejected/parked: NEMA+slip-ring rotation rig (motion replaces it), Hailo HAT
(RTX 4060 covers it), SBC compute (laptop wins), RAD-Moon 2 (cables not confirmed),
tactical IMU & higher-res cameras (poor quality/$ per research pass).
