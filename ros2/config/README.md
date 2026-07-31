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
- **`extrinsic_R` is identity by construction**, not derived from mount
  geometry. The IMU is physically co-mounted under the lidar with its axes
  parallel, so the mast tilt (**~58° from vertical, gravity-derived** — not the
  ~45° originally planned) cancels out. Do not re-derive it, and never smuggle a
  display yaw in — see `docs/imu_extrinsic.md`. `extrinsic_T` is the 7 cm
  tape-measured IMU→optical-origin offset up the spin axis.

Launch with an **absolute** config path — relative paths resolve against the
install tree and silently load the wrong file.

## GLIM (offline quality path)

`config_sensors.json`: set `T_lidar_imu` from the same extrinsic. CUDA build for
the RTX 4060. Notes land here at bring-up.
