# Repo update — 2026-07-26 ("First Light")

Upload these changed/new files over the existing repo (GitHub web: Add file →
Upload files, drag folders to preserve paths):

**NEW**
- `docs/zoox_quirks.md` — the critical doc: T1 recap, the zero-ranges
  laser_enable/FOV trap, factory-reset fix, NoiseFiltering note, safe posture
- `launch/rig.launch.py` — one-command startup (driver + bridge + record:=true)

**UPDATED**
- `README.md` — status checklist (bench ✔ converter ✔ IMU ✔ launch ✔), quirks links
- `docs/pinout.md` — CONFIRMED: T1 pair = orange/orange-white (Lemo 7/8)
- `docs/t1_ethernet.md` — confirmed BUELEC settings (100M, Master, orange pair)
- `hardware/bom.md` — statuses to current reality
- `firmware/imu_bridge/src/main.cpp` — v2 (IMU+GPS+PPS) with verified 0x27
  ODR registers (200 Hz; measured ~186 Hz, physics-checked)
- `ros2/imu_bridge_node/imu_bridge_node.py` — v2 (demux: Imu + NavSatFix + PPS)

**Still to add by hand:** first-light + Bobo screenshots into docs/img or README.
