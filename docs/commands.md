# Commands

The ones actually used on this rig, with absolute paths, in the order a
session tends to need them. Everything here is verified against the live
machine — if a command stops working, fix it here rather than remembering
the fix.

`~/.bashrc` already sources `/opt/ros/humble/setup.bash` and
`~/ros2_ws/install/setup.bash`, so no manual sourcing is needed in a normal
terminal.

---

## 1. Record a bag

Plug the XIAO in **before** launching — the bridge node hardcodes
`/dev/ttyACM0` and will not find it later.

```bash
ros2 launch ~/Desktop/rig_launch_v2.py record:=true
```

Hold the rig **dead still for 3–5 s** at the start; that window is what
gravity and bias init use, and every `bag_grav.py` check reads it. Then walk
the pass. Ctrl-C to stop. Bags land in `~/bags/run_<stamp>`.

Without `record:=true` the same command is a live-view session (driver +
RViz + bridge, no bag).

## 2. Replay, export and analyse — one command

```bash
~/pandar-40p/scripts/diagnostics/run_test.sh
```

Newest bag in `~/bags`, 2 cm voxel. To be explicit:

```bash
~/pandar-40p/scripts/diagnostics/run_test.sh ~/bags/run_20260801_014240 0.05
```

It runs the patch/config preflight, reports the mount signature, counts
recorded frames, clears the stale DDS shm, starts the mapper and waits for
the real `p_pre->lidar_type 2` banner, starts `save_map.py` before playback,
plays the bag, writes the PCD, stops the mapper cleanly so `mat_out.txt`
flushes, then prints the extrinsic analysis and the frame-drop count.

Playback is real time on purpose — replaying faster would change the
frame-drop behaviour being measured.

Outputs: `~/map_<bag>.pcd`, `~/mapper.log`, `~/save_map.log`,
`~/ros2_ws/src/FAST_LIO/Log/mat_out.txt`.

## 3. Individual diagnostics

All of these are standalone rclpy/sqlite scripts. Prefer them to the `ros2`
CLI, which is unreliable on this machine.

```bash
# which side of the 2026-07-31 reseat was this bag recorded on?
python3 ~/pandar-40p/scripts/diagnostics/bag_grav.py ~/bags/run_20260801_014240

# where did the online extrinsic estimate settle?
python3 ~/pandar-40p/scripts/diagnostics/analyze_ext.py \
    ~/ros2_ws/src/FAST_LIO/Log/mat_out.txt

# gravity from the LIVE imu (rig powered, bridge running, rig still)
python3 ~/pandar-40p/scripts/diagnostics/grav_vec.py

# is /lidar_points actually flowing, and what is in it?
python3 ~/pandar-40p/scripts/diagnostics/scan_peek.py

# accel/gyro unit sanity check
python3 ~/pandar-40p/scripts/diagnostics/imu_units.py
```

## 4. Run the mapper by hand

Only needed if `run_test.sh` is not what you want. The config path must be
**absolute** — a relative name silently loads `mid360.yaml` from the install
tree, and the map comes out wrong with no error.

```bash
ros2 launch fast_lio mapping.launch.py \
    config_file:=/home/lidar/ros2_ws/src/FAST_LIO/config/pandar40p.yaml
```

Proof it loaded the right file: the console prints `p_pre->lidar_type 2`.
A quiet console afterwards is normal — FAST-LIO2 prints almost nothing when
healthy, and RViz showing black is not a fault (the `camera_init` frame does
not exist until the mapper publishes TF).

Export a map from a hand-run mapper — start it **before** playback, Ctrl-C
**it** (not the mapper) after:

```bash
python3 ~/pandar-40p/scripts/diagnostics/save_map.py 0.02 ~/map.pcd
ros2 bag play ~/bags/run_20260801_014240
```

`pcd_save_en` in the config does **not** work — it produces nothing. Use
`save_map.py`.

## 5. Measure a map

**Use this, not CloudCompare's 3D view.** The map is stored in
`camera_init`, the IMU's orientation at t=0, and the IMU rides a ~45° mast
— so every exported map is tilted ~46.5°. CloudCompare's "Top" is not a
plan view and Height Ramp shades along a tilted axis. Measuring a doorway
in that view produced a 12% error that took hours to unpick.

```bash
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py ~/map_run_20260801_014240.pcd
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py map.pcd --slab 0.3 0.8
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py map.pcd --no-plot
```

It levels by the logged gravity vector, writes `<name>_level.pcd`, and
opens a top-down plan of a horizontal slab. `run_test.sh` now does the
levelling automatically at the end of every replay, so `_level.pcd`
already exists — this is only needed for one-off maps or to re-slice.

**Why any of this is necessary:** FAST-LIO's world frame `camera_init` is
the IMU's attitude at t = 0, and this rig's IMU rides a ~45° mast, so
every map is stored tilted (46–48°, depending on how the rig sat when you
started). There is no gravity-align option in this FAST-LIO — checked the
source. So it is corrected on export instead.

**To fix the live RViz view**, add `--tf` and it prints a ready static
transform for the run:

```bash
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py map.pcd --no-plot --tf
```

Run the `static_transform_publisher` line it gives you, then set RViz's
Fixed Frame to `map_level`. Display only — it never touches `extrinsic_R`,
which is the rule in docs/imu_extrinsic.md §2. The angle is per-run, so
regenerate it for each bag.

- **Click two points to measure.** It reports the click-to-click distance
  *and* the widest run with no returns between them — the second is the
  one to trust, because it finds the edges in the data instead of relying
  on your aim. Wall bands are 7–9 cm thick; clicking edges is hopeless.
- **Right-click always measures.** Left-click only measures when no
  toolbar tool is armed — and the magnifier stays armed after you zoom,
  which silently eats left-clicks. It tells you when that happens.
- `r` clears all measurements.

**If a map looks uniformly one colour in CloudCompare**, that is the height
ramp being stretched across outliers, not a broken ramp. The 2026-08-01
outdoor cloud spans 69 m (trees, plus a stray at +62 m) while 80 % of its
points sit in the bottom 8 m, so everything real lands in a sliver of the
scale. Write a pre-coloured cloud instead:

```bash
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py map.pcd --no-plot --color 0,8
python3 ~/pandar-40p/scripts/diagnostics/floorplan.py map.pcd --no-plot --color
```

The first clamps to 0–8 m; bare `--color` clamps to p1–p95 automatically.
Either writes `<name>_color.pcd` with real RGB, so CloudCompare shows it
correctly with no Height Ramp step at all.

## 6. View a map in CloudCompare

Open the `_level.pcd` that `floorplan.py` writes, not the raw one, or Top
and Height Ramp will both be wrong.

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    /snap/bin/cloudcompare.CloudCompare ~/map_run_20260801_014240_level.pcd
```

The env vars push it onto the NVIDIA card; the snap's Mesa cannot drive the
iGPU (0xa7a8). It will run without them, in software.

PCDs are XYZ-only and render **white** — fix with **Edit → Colors → Height
Ramp**. Switch off perspective projection before measuring. GTK theme
warnings on startup are noise.

## 7. IMU noise characterisation (Allan variance)

**Unplug the lidar first.** Its motor puts a 10 Hz line into the
accelerometer at 104× the noise floor, and stopping the ROS driver does
*not* stop it — the unit spins whenever powered.

```bash
~/pandar-40p/scripts/capture/allan_capture.sh          # start (in tmux)
~/pandar-40p/scripts/capture/allan_capture.sh status   # progress
~/pandar-40p/scripts/capture/allan_capture.sh stop     # finish
```

Runs in tmux session `avar`, so it survives a closed terminal or an SSH
drop — check it from your phone. Records only `/imu/data_raw`, ~350 MB/h.
Three hours minimum; overnight is much better, because bias instability
only appears at long averaging times.

Then:

```bash
python3 ~/pandar-40p/scripts/diagnostics/allan.py ~/bags/allan_<stamp> --plot ~/allan.png
python3 ~/pandar-40p/scripts/diagnostics/allan.py ~/bags/allan_<stamp> --max-hours 3
```

`allan_variance_ros` is **not** usable here — it is a catkin/ROS 1 package
depending on `rosbag` and `rospy`, with no ROS 2 branch upstream.
`allan.py` computes the same thing directly from the `.db3`.

Read the caveat it prints before pasting numbers into the config: FAST-LIO's
defaults sit far above true sensor noise on purpose, and operational noise
is 2–3× the quiet figures because the lidar is spinning while you map.

## 8. When ROS goes weird

The `ros2` CLI is genuinely unreliable here. In rough order of frequency:

```bash
# stale DDS shm after a hard kill — the usual culprit
pkill -f fastlio_mapping; pkill -f rviz2; pkill -f rosbag2_player
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

# daemon wedged (!rclpy.ok())
ros2 daemon stop && ros2 daemon start
```

`ros2 topic hz` blocks forever, so it never returns and chained commands
after it never run. Use `scan_peek.py` instead.

`ros2 bag info` Start/End come from the recorder's wall clock, not
`header.stamp`, so it **cannot** validate timestamp domains. Echo the header
stamps if that is the question.

## 9. Lidar web console

`http://192.168.1.201` — 600 rpm, return mode, PTP clock source.

```bash
python3 ~/pandar-40p/scripts/diagnostics/lidar_config.py   # read-only audit
```

**If every range reads zero, it is `NoiseFiltering`, not dead hardware.**
Confirmed 2026-08-01. One call fixes it, no factory reset (mind the
firmware's spelling):

```bash
curl -s "http://192.168.1.201/pandar.cgi?action=set&object=lidar_data&key=noise_filtring&value=0"
```

Full console reference, including every API object and the ones that erase
calibration: `docs/lidar_console.md`.

**The Azimuth FOV page's guilt is retracted.** It was blamed for the
zero-ranges fault for weeks, because the factory reset that recovered the
unit also cleared `NoiseFiltering` as a side effect. The 2026-08-01
reproduction pins the cause on `NoiseFiltering` alone, and no reset is
needed. Still no reason to go pressing Save there, but that is caution, not
a known fault.

**Do not use the old "`laser_enable` all-1, `laser_range` all-`[0,3600]`"
check** — it false-alarms on a healthy lidar. Those per-laser arrays are
only live when `angle_setting_method` is `1`; this unit runs method `0`,
where the global `lidar_range` governs and the per-laser arrays read all
zero *normally*. `lidar_config.py` reads the method first and says which
block is actually in force.

## 10. Git

Push straight to `main` — no PRs or side branches unless asked.

```bash
cd ~/pandar-40p && git add -A && git commit -m "..." && git push origin main
```

## 11. Remote access

Not project-specific; here because it is used often. Kept out of the repo
scripts on purpose — the helper lives at `~/.local/bin/claude-session.sh`.

```bash
tailscale status                 # is the rig on the tailnet?
claude-session.sh                # start/attach the tmux session
claude-session.sh resume         # same, but resumes the last conversation
claude-session.sh status | kill
```

From a phone: Tailscale app, then SSH to `lidar@lidar-scanner` (no key
needed — Tailscale SSH authenticates by tailnet identity), then
`claude-session.sh`. Detach with **Ctrl-b d**; the session keeps running.
