# FAST-LIO2 setup for the Pandar40P

Everything needed to get FAST-LIO2 producing a map from this rig, plus the
reasoning behind each non-obvious step. Verified working 2026-07-30.

---

## 1. Build

FAST-LIO's default branch is ROS 1. Use the ROS 2 branch.

```bash
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git checkout ROS2
```

It depends on `livox_ros_driver2` even for non-Livox sensors. Build that
first, with the ROS 2 edition flag:

```bash
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2
```

**Do not run `livox_ros_driver2`'s own `build.sh`.** It clobbers the
workspace environment and broke the Hesai driver build once. Recovery was
rebuilding the Hesai package with no extra arguments.

Then apply the patches in section 3 before building `fast_lio`.

---

## 2. Driver configuration

In `HesaiLidar_ROS_2.0/config/config.yaml`:

| Line | Setting | Value | Why |
|---|---|---|---|
| 15 | firetimes path | real path to `Pandar40P_Firetime Correction File.csv` | must not be a placeholder |
| 52 | `use_timestamp_type` | `1` | host receive time — see below |
| — | `multicast_ip_address` | `""` | empty string, not a placeholder |
| — | channel FOV filter path | `""` | empty string, not a placeholder |

### Why `use_timestamp_type: 1`

With `0`, message header stamps come from the lidar's own clock. With the
clock source set to GPS and no fix, that clock free-runs from a Y2K epoch —
header stamps read ~946688679 while the IMU reads ~1785376551. The two
streams live in different decades and can never be synchronized.

The failure is silent. FAST-LIO2 has a built-in `IMU and LiDAR not Synced`
warning, but it compares header stamps to each other in a way that did not
trigger here. The only symptom was `/Odometry` never publishing.

Setting `1` uses host receive time for both, costing some millisecond-scale
jitter. The quality upgrade later is PTP: run `ptp4l` as master, set the web
console's Clock Source to PTP, and `use_timestamp_type: 0` becomes correct
again.

**Verify before recording anything:**

```bash
ros2 topic echo /lidar_points  --field header.stamp --once
ros2 topic echo /imu/data_raw  --field header.stamp --once
```

Both `sec` values must be in the same era and within a second or two.

---

## 3. Required source patches

FAST-LIO2 cannot consume this driver's point cloud unmodified. Two changes.

### 3a. `src/preprocess.h` — point type must be double

The Hesai driver publishes `timestamp` as a `float64`. FAST-LIO2's
`velodyne_ros::Point` declares `float time`. Without this change the node
aborts with `Failed to find match for field 'time'`.

Line ~76:

```cpp
- float time;
+ double time;
```

And the registration macro (line ~81), which must also match the driver's
*field name*:

```cpp
- (float, time, time)
+ (double, time, timestamp)
```

`double` is not optional even ignoring the type check: a `float` holds ~7
significant digits, so an epoch value of 1.785e9 quantizes to roughly
128-second steps. The per-point timing would be destroyed before any later
fix could recover it.

### 3b. `src/preprocess.cpp` — normalize absolute per-point timestamps

The Hesai driver emits **absolute epoch seconds** in every point's
`timestamp` field. Measured directly:

```
header: 1785376550.411402
first : 1785376550.411402     <- identical to header
last  : 1785376550.511084
span  : 0.099682              <- one clean 10 Hz revolution
```

FAST-LIO2 reads that field as an offset from scan start. In
`velodyne_handler`:

```cpp
if (pl_orig.points[plsize - 1].time > 0) { given_offset_time = true; }
...
added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // units: ms
```

With `timestamp_unit: 0`, `time_unit_scale` is `1.e3f`, so `curvature`
becomes 1.785e9 × 1000 = **1.785e12 ms**. Then in `laserMapping.cpp`:

```cpp
// line ~406
lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
// line ~415
if (last_timestamp_imu < lidar_end_time) return false;
```

`lidar_end_time` lands roughly 56,000 years in the future, line 415 returns
false on every call, and `sync_packages` never assembles a measurement
group. Nothing is ever published. No error is printed.

**The patch.** Insert immediately after the empty-cloud guard at the top of
`velodyne_handler`, so every downstream branch sees relative times:

```cpp
  if (plsize == 0)
    return;

  /*** Hesai driver emits ABSOLUTE epoch seconds per point; FAST-LIO expects
       offsets from scan start. Normalize in place. No-op if already relative. ***/
  if (pl_orig.points[0].time > 1e8)
  {
    const double ts_base = pl_orig.points[0].time;
    for (int i = 0; i < plsize; i++)
    {
      pl_orig.points[i].time -= ts_base;
    }
  }

  pl_surf.reserve(plsize);
```

The `1e8` guard makes this survive a driver update: a genuine offset is
0–0.1, an epoch value is ~1.8e9, so there is no ambiguity. Using
`points[0].time` as the base is safe because it was measured equal to the
header stamp with a monotonic span.

A machine-readable version is in [`patches/`](../patches/).

Rebuild only this package:

```bash
cd ~/ros2_ws
colcon build --packages-select fast_lio
source install/setup.bash
```

---

## 4. Mapper configuration

`~/ros2_ws/src/FAST_LIO/config/pandar40p.yaml`, verified values:

```yaml
common:
    lid_topic: /lidar_points
    imu_topic: /imu/data_raw
    time_sync_en: false

preprocess:
    lidar_type: 2          # velodyne-style handler
    scan_line: 40
    timestamp_unit: 0      # seconds -> time_unit_scale 1.e3
    blind: 0.5             # ignore returns under 0.5 m (mount/self hits)
    point_filter_num: 3    # every 3rd point; 144k/frame is plenty

mapping:
    filter_size_surf: 0.5
    filter_size_map: 0.5
    extrinsic_est_en: true
    extrinsic_T: [ 0.0, 0.0, 0.07 ]
    extrinsic_R: [ 1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0 ]
```

See [imu_extrinsic.md](imu_extrinsic.md) for why the rotation is identity
and how that was verified.

### Launch with an absolute config path

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=/home/lidar/ros2_ws/src/FAST_LIO/config/pandar40p.yaml
```

A relative path resolves against the install tree, which silently loads a
different config for a newly added file. Confirm the console prints
`p_pre->lidar_type 2` — that is the proof the intended file loaded.

---

## 5. Verifying a run

FAST-LIO2 prints almost nothing during normal operation. A quiet console is
**not** evidence of failure, and RViz's default preset only shows
`/cloud_registered` and `/Path` — both produced by the mapper — so a black
view means no output, not no data.

Use the scripts in [`scripts/diagnostics/`](../scripts/diagnostics/) instead. Recommended order:

```bash
python3 scripts/diagnostics/imu_units.py    # gyro must be rad/s, accel ~9.8 m/s^2
python3 scripts/diagnostics/grav_vec.py     # per-axis gravity; confirms mounting
python3 scripts/diagnostics/scan_peek.py    # /cloud_registered extents and point count
python3 scripts/diagnostics/save_map.py     # accumulate and export a PCD
```

Healthy values for this rig:

| Check | Expected |
|---|---|
| `/lidar_points` rate | 10 Hz |
| `/imu/data_raw` rate | ~200 Hz |
| points per registered scan | ~47,900 (144k ÷ `point_filter_num` 3) |
| single-scan extents indoors | metres in every axis, floor to ceiling |
| peak \|gyro\| while walking | 0.5–1.5 (rad/s) |
| mean \|accel\| | ~9.8–9.9 m/s² |

---

## 6. Known issues

### The `ros2` CLI is unreliable on this machine

Three distinct failures during one session:

- `RTPS_TRANSPORT_SHM Error: Failed init_port ... open_and_lock_file failed` —
  stale Fast-DDS shared-memory files. Clean with:
  ```bash
  pkill -f ros2; pkill -f rviz2; sleep 2
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
  ```
- `xmlrpc.client.Fault: <class 'RuntimeError'>:!rclpy.ok()` — the CLI
  daemon died. `ros2 daemon stop && ros2 daemon start`.
- `ros2 topic hz` blocks, so chaining two of them runs only the first.

Small standalone `rclpy` scripts avoid all of this and are the preferred
diagnostic path. That is why `scripts/diagnostics/` exists.

### `pcd_save_en` produces nothing

`pcd_save_en: true` under `pcd_save:` did not write a file. `PCD/` contained
only the 2-byte placeholder the upstream repo ships to keep the directory in
git. Root cause not fully established — the config is read at launch, so an
edit made while the node was running would not take, and a SIGTERM (as sent
by `run_test.sh`'s cleanup trap) may skip the save path entirely.

Use [`scripts/diagnostics/save_map.py`](../scripts/diagnostics/save_map.py) instead. It subscribes to
`/cloud_registered`, voxel-downsamples, and writes a binary PCD on Ctrl+C,
with no dependence on FAST-LIO2's shutdown behaviour.

### `ros2 bag info` does not validate timestamps

The `Start:` and `End:` fields come from rosbag2's own receive clock, not
from `header.stamp`. A bag full of Y2K-stamped messages recorded today still
reports today's date. Always check header stamps directly.

---

## 7. Reference results

First successful map, 2026-07-30:

| | |
|---|---|
| Bag | `run_20260729_215544`, 87.1 s |
| Lidar frames | 871 (10.0 Hz) |
| IMU messages | 17,530 (201 Hz) |
| Raw registered points | 25,666,477 |
| After 5 cm voxel | 261,947 |
| After 2 cm voxel | 1,877,815 |
| Doorway measured | 0.77 m — **retracted**, see below |

The ~99% reduction at 5 cm is arithmetic, not data loss: a house interior is
roughly 500–600 m² of surface, which at 5 cm is about 220k occupied voxels.
The other 25 million points are the same surfaces seen repeatedly.

**Two caveats on this table, both found later:**

- **The doorway figure is retracted.** 0.813 m is a 32-inch nominal for what
  the tape says is a 28-inch door, and every doorway measurement was taken
  in a ~46° tilted CloudCompare view. Scale is verified independently
  against a taped ceiling at **−0.53 %**. See docs/imu_extrinsic.md §6.
- **This run used a third to a half of its own data.** The mapper subscribed
  with `SensorDataQoS()` — BEST_EFFORT, depth 5 — and silently dropped most
  of a ~35 MB/s PointCloud2 stream. 483 of 871 frames reached it here. Fixed
  2026-08-01 (99.4 % after), and metric accuracy improved measurably with
  it. Every point count above is therefore a lower bound.

---

## 8. Next steps

- [x] **Frame drop confirmed and fixed** — reliable QoS in place of
  `SensorDataQoS()`; 99.4 % of frames processed, and floor→ceiling accuracy
  went −1.92 % → −0.53 % with the recovered data
- [x] **`extrinsic_est_en` checked against the hand measurement.** R is
  resolved: identity within 0.65° on every axis once the mount was straight.
  T is *not* observable in this data — it has never moved more than ~2 mm
  from its initialization, including from a value ~6 cm wrong — so it rests
  on the tape alone: `[-0.057, -0.023, 0.047]`
- [x] **IMU noise characterised** — 8.58 h static, lidar unplugged. Every
  axis beats the datasheet on white noise. `allan_variance_ros` is ROS 1
  only; `scripts/diagnostics/allan.py` computes it from the `.db3`.
  **Do not paste the derived covariances into the config** — FAST-LIO's
  defaults sit 5–8 orders higher on purpose, absorbing un-modelled error
- [x] **Outdoor capture with loop closure** — 234 m sidewalk loop,
  **0.55 % drift**, only 10 cm of it vertical
- [ ] PTP time sync, then revert `use_timestamp_type` to `0` — **deferred**.
  No PTP hardware on any interface on this machine, and software PTP buys
  ~0.4 mm at walking pace against 1,277 mm of measured drift. It is
  correctness, not accuracy, and type 0 without a solid lock re-creates the
  silent timestamp-domain failure that cost days in July
- [ ] Camera aim → panel bond → intrinsics → lidar-camera calibration
- [ ] Offline chain: GLIM → HBA → dynamic removal → colorize
