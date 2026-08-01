#!/usr/bin/env python3
"""Gravity-level a FAST-LIO2 map, then measure distances on a floor plan.

Why this exists: FAST-LIO2's world frame (camera_init) is the IMU's
orientation at t=0, and this rig's IMU rides a mast tilted ~45 deg. The
exported map is therefore stored tilted ~46.5 deg from vertical. In
CloudCompare that means "Top" is not a plan view, walls do not stand up on
screen, and Edit > Colors > Height Ramp shades along a tilted axis instead
of by height. Measuring anything in that view is needlessly hard.

The true vertical is not a guess: FAST-LIO2 estimates the gravity vector
and logs it to Log/mat_out.txt (columns 22-24, verified against
laserMapping.cpp:1103). This reads it, rotates the cloud so gravity points
along -Z, and then:

  * writes <name>_level.pcd, which CloudCompare treats normally --
    Top really is a plan view and Height Ramp really is height
  * opens an interactive top-down plan of a horizontal slab, where
    clicking two points prints the distance between them

usage:
    floorplan.py ~/map_run_20260801_014240.pcd
    floorplan.py map.pcd --slab 0.9 1.5      # slab height above floor (m)
    floorplan.py map.pcd --no-plot           # just write the levelled PCD
    floorplan.py map.pcd --grav X Y Z        # override the logged gravity
"""
import argparse
import math
import os
import sys

import numpy as np

MAT = os.path.expanduser("~/ros2_ws/src/FAST_LIO/Log/mat_out.txt")


def read_pcd(path):
    """Read the XYZ binary PCD that save_map.py writes."""
    with open(path, "rb") as fh:
        raw = fh.read()
    end = raw.find(b"DATA binary\n")
    if end < 0:
        sys.exit(f"{path}: not a binary PCD written by save_map.py")
    header = raw[:end].decode("ascii", "replace")
    n = None
    for line in header.splitlines():
        if line.startswith("POINTS"):
            n = int(line.split()[1])
    if n is None:
        sys.exit(f"{path}: no POINTS field in header")
    body = raw[end + len(b"DATA binary\n"):]
    pts = np.frombuffer(body, dtype=np.float32, count=n * 3).reshape(-1, 3)
    return pts.astype(np.float64)


def write_pcd(path, pts):
    pts = pts.astype(np.float32)
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(pts.tobytes())


def gravity_from_log(path=MAT, tail=50):
    """Mean of the last `tail` logged gravity vectors (cols 22-24)."""
    rows = []
    try:
        with open(path) as fh:
            for line in fh:
                f = line.split()
                if len(f) >= 25:
                    try:
                        rows.append([float(f[22]), float(f[23]), float(f[24])])
                    except ValueError:
                        pass
    except OSError:
        return None
    if not rows:
        return None
    g = np.mean(np.array(rows[-tail:]), axis=0)
    if not 8.5 < np.linalg.norm(g) < 11.0:
        print(f"  WARN logged gravity has |g| = {np.linalg.norm(g):.2f}, "
              "which is not gravity. Levelling may be wrong.")
    return g


def rotation_bringing_to_down(g):
    """Minimal rotation taking unit vector g to (0,0,-1). Adds no yaw."""
    a = np.asarray(g, dtype=float)
    a /= np.linalg.norm(a)
    b = np.array([0.0, 0.0, -1.0])
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-12:                      # already aligned (or exactly inverted)
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def write_pcd_rgb(path, pts, lo, hi, cmap="turbo"):
    """Write an XYZ+RGB PCD coloured by height, clamped to [lo, hi].

    CloudCompare's Height Ramp stretches across the FULL z range, and an
    outdoor cloud spans ~69 m because of trees and strays while 80 % of the
    points sit in the bottom 8 m. Everything you care about then lands in a
    sliver of the colour scale and the result looks uniform. Clamping is the
    whole point of this function.
    """
    import matplotlib.cm as cm
    z = np.clip((pts[:, 2] - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    rgb = (np.asarray(cm.get_cmap(cmap)(z))[:, :3] * 255).astype(np.uint32)
    packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    out = np.empty((len(pts), 4), dtype=np.float32)
    out[:, :3] = pts[:, :3].astype(np.float32)
    out[:, 3] = packed.astype(np.uint32).view(np.float32)
    n = len(out)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n"
        f"COUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(out.tobytes())


def quat_from_matrix(R):
    """(x, y, z, w) from a rotation matrix, for a TF static publisher."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
        x, y, z = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def empty_run(pts, p0, p1, corridor=0.06):
    """Widest gap in the returns along the segment p0->p1.

    Clicking the exact edge of a wall is hopeless: the bands are 7-9 cm
    thick and the jamb ends are ragged, so the answer moves by centimetres
    depending on aim. Instead, take a thin corridor along the line the user
    drew, project the points onto it, and report the widest run with no
    returns at all. The edges then come from the data.
    """
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    v = p1 - p0
    L = np.linalg.norm(v)
    if L < 1e-6:
        return None
    v /= L
    n = np.array([-v[1], v[0]])
    rel = pts[:, :2] - p0
    s = rel @ v
    t = np.abs(rel @ n)
    keep = (t <= corridor) & (s >= 0) & (s <= L)
    s = np.sort(s[keep])
    if len(s) < 2:
        return None
    d = np.diff(s)
    return float(d.max()) if d.size else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("--slab", nargs=2, type=float, default=[0.9, 1.5],
                    metavar=("LO", "HI"),
                    help="slab height above the floor, metres (default 0.9 1.5)")
    ap.add_argument("--grav", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "Z"), help="override logged gravity")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--tf", action="store_true",
                    help="also print an RViz static-TF command for this run")
    ap.add_argument("--color", nargs="?", const="auto", default=None,
                    metavar="LO,HI",
                    help="also write <name>_color.pcd, height-coloured and "
                         "CLAMPED. Default clamps to p1..p95 of height, which "
                         "is what makes an outdoor cloud actually readable.")
    ap.add_argument("--mat", default=MAT, help="path to mat_out.txt")
    args = ap.parse_args()

    pts = read_pcd(args.pcd)
    print(f"read {len(pts):,} points from {args.pcd}")

    g = np.array(args.grav) if args.grav else gravity_from_log(args.mat)
    if g is None:
        sys.exit(f"no gravity found in {args.mat}; pass --grav X Y Z")
    gu = g / np.linalg.norm(g)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -gu[2]))))
    print(f"gravity in map coords: ({gu[0]:+.4f}, {gu[1]:+.4f}, {gu[2]:+.4f})")
    print(f"  map is tilted {tilt:.1f} deg from vertical -- levelling")

    R = rotation_bringing_to_down(gu)
    lev = pts @ R.T

    floor = float(np.percentile(lev[:, 2], 1.0))
    lev[:, 2] -= floor
    print(f"  floor set at the 1st percentile of height; "
          f"span now {lev[:,2].min():.2f} .. {lev[:,2].max():.2f} m")

    out = os.path.splitext(args.pcd)[0] + "_level.pcd"
    write_pcd(out, lev)
    print(f"wrote {out}")
    print("  in CloudCompare: Top is now a real plan view, and Height Ramp")
    print("  really is height. Use orthographic (not perspective) to measure.")

    if args.tf:
        q = quat_from_matrix(R)
        print("\nRViz: publish this, then set Fixed Frame to 'map_level'.")
        print("  (the angle depends on how the rig sat at t=0, so it is")
        print("   per-run -- regenerate it for each bag)")
        print(f"\nros2 run tf2_ros static_transform_publisher \\\n"
              f"  --x 0 --y 0 --z 0 "
              f"--qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} \\\n"
              f"  --frame-id map_level --child-frame-id camera_init")

    if args.color:
        if args.color == "auto":
            clo, chi = np.percentile(lev[:, 2], [1, 95])
        else:
            clo, chi = (float(x) for x in args.color.split(","))
        cout = os.path.splitext(args.pcd)[0] + "_color.pcd"
        write_pcd_rgb(cout, lev, clo, chi)
        print(f"\nwrote {cout}")
        print(f"  height-coloured, clamped {clo:.2f} .. {chi:.2f} m")
        print("  open THIS one in CloudCompare -- no Height Ramp needed, and")
        print("  it will not be washed out by tall trees or stray points.")

    lo, hi = args.slab
    sel = lev[(lev[:, 2] >= lo) & (lev[:, 2] <= hi)]
    print(f"\nslab {lo:.2f}-{hi:.2f} m above floor: {len(sel):,} points")
    if len(sel) == 0:
        sys.exit("empty slab -- try a different --slab range")

    if args.no_plot:
        return

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(sel[:, 0], sel[:, 1], s=0.12, c="#1a1a1a", linewidths=0,
               rasterized=True)
    ax.set_aspect("equal")
    ax.grid(True, which="major", lw=0.5, alpha=0.35)
    ax.grid(True, which="minor", lw=0.3, alpha=0.18)
    ax.minorticks_on()
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{os.path.basename(args.pcd)}  slab {lo:.2f}-{hi:.2f} m\n"
                 "click two points to measure  |  r = reset  |  "
                 "toolbar magnifier to zoom")

    picks = []
    artists = []

    def on_click(ev):
        if ev.inaxes is not ax or ev.xdata is None:
            return
        # Right-click always measures. Left-click measures only when no
        # toolbar tool is armed -- and the magnifier STAYS armed after you
        # zoom, which silently ate every measuring click the first time this
        # was used. Say so instead of doing nothing.
        mode = getattr(getattr(fig.canvas, "toolbar", None), "mode", "")
        if ev.button != 3 and mode:
            print(f"  ({mode} is active -- right-click to measure, or click "
                  "the toolbar button to disarm it)", flush=True)
            return
        picks.append((ev.xdata, ev.ydata))
        artists.append(ax.plot(ev.xdata, ev.ydata, "o", ms=6,
                               color="#d1495b")[0])
        if len(picks) % 2 == 0:
            (x0, y0), (x1, y1) = picks[-2], picks[-1]
            d = math.hypot(x1 - x0, y1 - y0)
            gap = empty_run(sel, (x0, y0), (x1, y1))
            artists.append(ax.plot([x0, x1], [y0, y1], "-", lw=1.6,
                                   color="#d1495b")[0])
            label = f"click {d:.3f} m ({d/0.0254:.1f} in)"
            if gap is not None:
                label += f"\ngap {gap:.3f} m ({gap/0.0254:.1f} in)"
            artists.append(ax.annotate(
                label, ((x0 + x1) / 2, (y0 + y1) / 2), color="#d1495b",
                fontsize=10, fontweight="bold",
                bbox=dict(fc="white", ec="#d1495b", alpha=0.85, pad=2)))
            print(f"  click-to-click {d:.4f} m = {d/0.0254:.2f} in", flush=True)
            if gap is not None:
                print(f"  measured gap   {gap:.4f} m = {gap/0.0254:.2f} in"
                      "   <- edges found in the data, not by your aim",
                      flush=True)
        fig.canvas.draw_idle()

    def on_key(ev):
        if ev.key == "r":
            for a in artists:
                a.remove()
            artists.clear()
            picks.clear()
            fig.canvas.draw_idle()
            print("  cleared")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    print("\nclick two points to measure; 'r' clears; zoom with the toolbar")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
