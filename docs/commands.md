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

## 5. View a map

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    /snap/bin/cloudcompare.CloudCompare ~/map_run_20260801_014240.pcd
```

The env vars push it onto the NVIDIA card; the snap's Mesa cannot drive the
iGPU (0xa7a8). It will run without them, in software.

PCDs from `save_map.py` are XYZ-only and render **white** — fix with
**Edit → Colors → Height Ramp**. GTK theme warnings on startup are noise.

## 6. When ROS goes weird

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

## 7. Lidar web console

`http://192.168.1.201` — 600 rpm, return mode, PTP clock source.

**Never press Save on the Azimuth FOV page.** It can persist
`laser_enable` all-zero and zero-width `laser_range` windows, leaving the
unit spinning and streaming with every range `0x0000`. Manual repair failed
last time; only a factory reset fixed it. Verify via Device Log JSON:
`laser_enable` all-1, `laser_range` all-`[0,3600]`.

## 8. Git

Push straight to `main` — no PRs or side branches unless asked.

```bash
cd ~/pandar-40p && git add -A && git commit -m "..." && git push origin main
```

## 9. Remote access

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
