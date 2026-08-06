# Bill of Materials & Status

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | Hesai Pandar40P #1 | teardown donor | T1 fleet variant; BCM89811 confirmed; optics/parts salvage |
| 2 | Hesai Pandar40P #2 | **VERIFIED WORKING** | PA40-Zoox fw 2.20.17; rings in RViz 2026-07-26. The zero-ranges scare was `NoiseFiltering=1`, not a fault — reproduced and reversed on demand 2026-08-01, no factory reset needed |
| 3 | BUELEC 100/1000Base-T1-TX-E converter | **in service** | 100M + MASTER + orange pair = link; mounted on mast |
| 4 | Laptop i7-12650H / RTX 4060 8 GB / 32 GB, GbE | owned | Ubuntu 22.04 dual boot; capture + SLAM + post |
| 5 | ICM-42688-P breakout (Teyleten) | **in service** | wired to XIAO (SPI), 0x27 regs (DS-000347 verified), **~201 Hz measured**, physics checked. Reseated 2026-07-31 (first mount was 14.4° crooked). Allan variance done: beats datasheet white noise on every axis |
| 6 | MPU6050 | retired to drawer | ICM went straight in; never needed |
| 7 | XIAO ESP32-S3 | **in service** | v2 firmware. GPS wired to **D5** at 9600 8N1 and working. `PIN_PPS`=D4 set to `INPUT_PULLDOWN` 2026-08-01, which killed the long-standing "PPS flood" (a floating pin, never a GPS quirk) |
| 8 | WHEELTEC/FDI N100 IMU | candidate | 9-axis, 400 Hz raw, USB, ROS2 driver — would replace 5+7 |
| 9 | ELP-USB3DGS1200P01-H120 dual GS cameras **×2** | on rig (mockup) | in printed cases on ball mounts; aim→bond→calibrate pending; no external trigger pad (email ELP). **Bandwidth measured**: both sustain 3200×1200 @ 15 fps; 30 fps fails on USB 2.0 isochronous reservation. No measurable effect on IMU timing |
| 10 | u-blox M10 GNSS | in service | L1-only, metre-class. Feeds `/gps/fix` via the XIAO. Cannot do RTK — the InCORS streams are L1+L2 (`carrier=2`) |
| 11 | 12 V flooded lead-acid deep cycle + fuse | **in service** | powers the whole rig. Monitored by #17. Fuse AT THE BATTERY TERMINAL — a deep cycle will push hundreds of amps into a short |
| 12 | Jogging stroller (pneumatic tires) | **in service** | air tires were the vibration fix, and it is settled by measurement: 234 m sidewalk run peaked at 32.59 m/s² = 42 % of ±8 g, zero samples above 50 %. **Rubber isolators not needed** |
| 13 | Mast + brackets | **built, in service** | welded, plug-AFT, **~45–47° from vertical** (Klein gauge ~47°, post-reseat gravity 44.5°). The old "~58°" figure was a crooked IMU, not the mast |
| 14 | Checkerboard target | owned | camera intrinsics |
| 15 | **Quectel LG290P RTK module** | **in hand, working 2026-08-06** | Bought instead of the planned ZED-F9P. Rover mode, 10 Hz, 460800 8N1 over a CH343 USB-serial. Tracks GPS+GLONASS+Galileo+BeiDou+QZSS+NavIC — verified from `PQTMCFGCNST` and the NMEA talkers. RTK float in 1.8 s on free InCORS `MSM4_VRS`; not yet fixed, not yet mounted. See docs/rtk_gnss.md |
| 16 | L1/L2 GNSS antenna for #15 | **needed** | Must become the highest thing on the rig — the Pandar40P is a spinning metal cylinder and an antenna beside or below it buys occlusion and multipath |
| 17 | INA226 + Waveshare ESP32-C6-LCD-1.47 battery monitor | **firmware built and flashed; awaiting wiring** | VCC→3V3(OUT), GND→GND, SDA→GP18, SCL→GP19, shunt high-side in the battery positive. Hardware ALERT on GP20 at 12.0 V drives the RGB LED, so a hung sketch cannot swallow it. Amber 12.2 V = 50 % DoD |
| 18 | M5Stack Tab5 (ESP32-P4 + C6) | owned, unused | intended WiFi dashboard for `rig_status_node` JSON on :8080. Needs a P4 hello-world first to check the toolchain |

Rejected/parked: NEMA+slip-ring rotation rig (motion replaces it), Hailo HAT
(RTX 4060 covers it), SBC compute (laptop wins), RAD-Moon 2 (cables not confirmed),
tactical IMU & higher-res cameras (poor quality/$ per research pass).
