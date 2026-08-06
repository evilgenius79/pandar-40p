# ROS 2 configs

## Hesai driver (HesaiLidar_ROS_2.0 — official, supports Pandar40P on Humble)

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd .. && colcon build --symlink-install
```

The live working config is committed here as **`hesai_config.yaml`** (a
byte-for-byte copy of `~/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml`).
Key settings in it:
- `device_ip_address`: 192.168.1.201 (factory), UDP 2368
- per-unit **correction file** and **firetimes file** paths set to the real
  Pandar40P CSVs — the sensor auto-imports its own calibration, but loading the
  right files in the driver matters for registration quality (see build_guide
  Quality Tier 1).
- `use_timestamp_type: 1` (host receive time). Type 0 uses the sensor clock,
  which free-runs from Y2K epoch without a GPS fix and silently starves
  FAST-LIO2 — see `docs/fastlio_setup.md`.

Launch: `ros2 launch hesai_ros_driver start.py` → point cloud in RViz2.

## FAST-LIO2 (live view SLAM)

Done and working — the live config is committed here as
**`fastlio_pandar40p.yaml`** (copy of
`~/ros2_ws/src/FAST_LIO/config/pandar40p.yaml`). It rides the Velodyne parse
path (`lidar_type: 2`, `scan_line: 40`, `timestamp_unit: 0`) with
`lid_topic: /lidar_points`, `imu_topic: /imu/data_raw`.

Two things this config depends on, both non-obvious:
- **Source patches are mandatory** — `patches/fastlio_pandar40p.patch` widens
  the per-point time field to `double` and rebases absolute epoch per-point
  timestamps to frame-relative. Without them `sync_packages` never fires and
  the mapper fails silently. See `docs/fastlio_setup.md`.
- **`extrinsic_R` is identity — and that is now earned by measurement, not
  asserted.** The IMU is physically co-mounted under the lidar with its axes
  parallel, so the mast tilt (**~45–47° from vertical**: Klein gauge ~47°,
  post-reseat gravity 44.5°, mast built for ~45°) cancels out. Do not
  re-derive it, and never smuggle a display yaw in — see
  `docs/imu_extrinsic.md`.
  - **"By construction" was false for a week.** The first co-mount sat 14.4°
    crooked about X and nobody checked, because the phrase was treated as a
    proof. It was reseated 2026-07-31; on the first bag afterwards the online
    estimator lands at roll +0.64°, pitch +0.38°, yaw −0.26°, against −6.1° of
    roll before. Re-verify with gravity after **any** mount work.
  - An older note here gave the tilt as "~58° from vertical, gravity-derived".
    That was the crooked IMU talking, not the mast. Retracted.
- **`extrinsic_T` is `[-0.057, -0.023, 0.047]`**, tape-measured 2026-08-01:
  the IMU sits 5.7 cm left, 2.3 cm aft and 4.7 cm below the lidar centre, and
  the config wants the negation because it expresses the lidar in IMU axes
  (`R*p_lidar + T`, verified in `laserMapping.cpp:895` → `IMU_Processing.hpp:327`).
  The old "7 cm up the spin axis" figure predates the reseat.
  **T rests on the tape alone** — it is effectively unobservable in this data,
  and the estimator has never moved it more than ~2 mm from wherever it was
  initialized, including on a run where it started ~6 cm wrong.

Launch with an **absolute** config path — relative paths resolve against the
install tree and silently load the wrong file.

## GLIM (offline quality path)

`config_sensors.json`: set `T_lidar_imu` from the same extrinsic. CUDA build for
the RTX 4060. Notes land here at bring-up.
