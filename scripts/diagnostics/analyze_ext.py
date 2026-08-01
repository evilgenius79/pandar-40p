#!/usr/bin/env python3
"""Parse FAST-LIO2 Log/mat_out.txt and report the extrinsic estimate over time.

Column layout, from laserMapping.cpp:1103 (fout_out <<):
  0      time = lidar_beg_time - first_lidar_time
  1..3   euler_cur (deg)
  4..6   pos
  7..9   ext_euler  = SO3ToEuler(offset_R_L_I), DEGREES (use-ikfom.hpp:105, *57.3)
  10..12 offset_T_L_I           <-- the lidar->IMU translation we care about
  13..15 vel
  16..18 bg
  19..21 ba
  22..   grav, feats_undistort size
"""
import os
import re
import sys
import statistics as st

path = sys.argv[1]
tag = sys.argv[2] if len(sys.argv) > 2 else path

CFG = os.path.expanduser("~/ros2_ws/src/FAST_LIO/config/pandar40p.yaml")


def declared_extrinsic_T(cfg=CFG):
    """Read extrinsic_T out of the live config.

    Hardcoding it here was a real bug: the config moved to
    [-0.057,-0.023,0.047] on 2026-08-01 and this script kept reporting
    deltas against the retired [0,0,0.07], which made the comparison
    silently meaningless. Read it, don't remember it.
    """
    try:
        with open(cfg) as fh:
            m = re.search(r"^\s*extrinsic_T:\s*\[([^\]]+)\]", fh.read(), re.M)
        if not m:
            return None
        vals = [float(x) for x in m.group(1).split(",")]
        return vals if len(vals) == 3 else None
    except OSError:
        return None

rows = []
for line in open(path):
    f = line.split()
    if len(f) < 13:
        continue
    try:
        rows.append([float(x) for x in f[:13]])
    except ValueError:
        continue

if not rows:
    sys.exit(f"no parsable rows in {path}")

t = [r[0] for r in rows]
rx = [r[7] for r in rows]
ry = [r[8] for r in rows]
rz = [r[9] for r in rows]
tx = [r[10] for r in rows]
ty = [r[11] for r in rows]
tz = [r[12] for r in rows]

print(f"=== {tag} ===")
print(f"rows={len(rows)}  t={t[0]:.2f}..{t[-1]:.2f} s")
print()

# trajectory: decile snapshots
print("  t(s)     ext_R roll/pitch/yaw (deg)        offset_T_L_I x/y/z (m)")
n = len(rows)
for i in range(0, n, max(1, n // 10)):
    print(f"  {t[i]:6.1f}   {rx[i]:8.4f} {ry[i]:8.4f} {rz[i]:8.4f}    "
          f"{tx[i]:8.5f} {ty[i]:8.5f} {tz[i]:8.5f}")
print(f"  {t[-1]:6.1f}   {rx[-1]:8.4f} {ry[-1]:8.4f} {rz[-1]:8.4f}    "
      f"{tx[-1]:8.5f} {ty[-1]:8.5f} {tz[-1]:8.5f}   <- final")
print()

# stability over the last quarter of the run
q = max(1, n // 4)
def stats(name, v, unit):
    tail = v[-q:]
    m, sd = st.mean(tail), (st.pstdev(tail) if len(tail) > 1 else 0.0)
    print(f"  {name:>6}  mean={m:9.5f} {unit}  sd={sd:8.5f}  "
          f"min={min(tail):9.5f}  max={max(tail):9.5f}  drift(last-first)={tail[-1]-tail[0]:+9.5f}")

print(f"last-quarter stability (n={q} frames, t>={t[-q]:.1f}s):")
for nm, v, u in (("R roll", rx, "deg"), ("R pitch", ry, "deg"), ("R yaw", rz, "deg"),
                 ("T x", tx, "m"), ("T y", ty, "m"), ("T z", tz, "m")):
    stats(nm, v, u)
print()
decl = declared_extrinsic_T()
if decl is None:
    print(f"could not read extrinsic_T from {CFG} -- skipping the delta")
else:
    print(f"declared in config: extrinsic_T = {decl} m, extrinsic_R = identity")
    dT = [tx[-1] - decl[0], ty[-1] - decl[1], tz[-1] - decl[2]]
    print(f"final delta vs declared:  dT = [{dT[0]:+.5f}, {dT[1]:+.5f}, {dT[2]:+.5f}] m")
    print(f"                          dR = [{rx[-1]:+.4f}, {ry[-1]:+.4f}, {rz[-1]:+.4f}] deg")
    print()
    moved = max(abs(v) for v in dT)
    if moved < 0.002:
        print(f"  NOTE T moved at most {moved*1000:.1f} mm from where it was")
        print("       initialized. On this rig T has never been observably")
        print("       estimated -- see CLAUDE.md, the retracted 'translation")
        print("       confirms the tape measure' bullet. Do not read the")
        print("       agreement below as confirmation of the hand measurement.")
