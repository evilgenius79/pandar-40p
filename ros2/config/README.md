# ROS 2 configs

## Hesai driver (HesaiLidar_ROS_2.0 — official, supports Pandar40P on Humble)

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd .. && colcon build --symlink-install
```

Edit its `config/config.yaml`:
- `lidar_type` / model: Pandar40P
- `device_ip_address`: whatever `find_lidar.sh` reported (factory: 192.168.1.201)
- ensure per-unit **correction file** and **firetimes file** paths are set —
  the sensor auto-imports its own calibration, but loading the right files in the
  driver matters for registration quality (see build_guide Quality Tier 1).

Launch: `ros2 launch hesai_ros_driver start.py` → point cloud in RViz2.

## FAST-LIO2 (live view SLAM)

Known integration task (one evening): stock configs cover Livox/Velodyne/Ouster
point layouts; the Hesai driver's PointCloud2 uses per-point **timestamp** +
**ring** fields under Hesai's field names. Community Pandar configs exist —
adapt `velodyne.yaml`:
- point time field name + units (Hesai: absolute double seconds per point)
- `scan_line: 40`
- extrinsic_T / extrinsic_R: lidar↔IMU from the mount geometry (45° tilt!)
- IMU topic: `/imu/data_raw`

`pandar40p_fastlio.yaml` will be committed here once first bags exist to test
against.

## GLIM (offline quality path)

`config_sensors.json`: set `T_lidar_imu` from the same extrinsic. CUDA build for
the RTX 4060. Notes land here at bring-up.
