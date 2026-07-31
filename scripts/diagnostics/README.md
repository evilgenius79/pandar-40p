# Diagnostic and export scripts

Small standalone `rclpy` tools. They exist because the `ros2` CLI proved
unreliable on the rig laptop (stale `/dev/shm` Fast-DDS locks, a daemon
that dies with `!rclpy.ok()`, and `topic hz` blocking); these bypass the
CLI plumbing entirely.

All assume the workspace is sourced. None require the mapper unless noted.

| Script | What it does | When |
|---|---|---|
| `imu_units.py` | Peak \|gyro\| and mean \|accel\| over 600 samples; flags deg/s-vs-rad/s and g-vs-m/s² | New firmware, new bridge, or any rotation-related divergence. Run during the *moving* part of a bag |
| `grav_vec.py` | Averages 400 accel samples into a per-axis gravity vector | After any IMU remount, rig dead still. See `docs/imu_extrinsic.md` for interpreting it |
| `scan_peek.py` | Point count and XYZ extents of 5 scans from `/cloud_registered` | Mapper running. Splits "registering garbage" from "registering nothing" in one look |
| `save_map.py` | Accumulates `/cloud_registered`, voxel-downsamples, writes binary PCD on Ctrl+C | The map export path. FAST-LIO2's own `pcd_save_en` produced nothing on this rig |
| `run_test.sh` | One-shot: config checks → SHM cleanup → mapper launch → bag replay → `scan_peek` | Regression test after config or source changes. Edit `BAG=` first. Do **not** rely on it for PCD export — its cleanup trap SIGTERMs the mapper |

## save_map.py usage

```bash
python3 save_map.py [voxel_m] [output.pcd]
python3 save_map.py                 # 0.05 m -> ~/map.pcd
python3 save_map.py 0.02 ~/map_2cm.pcd
```

Start it **before** playing the bag; Ctrl+C it **after** the bag ends (it
is the accumulator holding the points — killing the mapper first loses
nothing). The downsample sorts tens of millions of rows and sits silent
for a minute or two before writing. Sizing reference from the first run:
25.7M raw → 262k at 5 cm → 1.88M at 2 cm.

Output is XYZ-only, so CloudCompare renders it white: Edit → Colors →
Height Ramp fixes that.
