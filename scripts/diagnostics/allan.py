#!/usr/bin/env python3
"""Overlapping Allan deviation for a long static /imu/data_raw capture.

Replaces allan_variance_ros, which cannot be used here: it is a catkin
package depending on rosbag/rospy and using ros::NodeHandle and
rosbag::View throughout, with no ROS 2 branch upstream. Rather than port
it or build an unvetted fork into ~/ros2_ws (which has broken the hesai
package once already), this reads the ROS 2 .db3 directly with sqlite3,
the same way bag_grav.py does.

usage:
    allan.py ~/bags/allan_20260801_030453
    allan.py <bag> --plot ~/allan.png
    allan.py <bag> --max-hours 6        # cap how much is read

Reports, per axis:
  N  white noise / random walk   -- the tau^-1/2 asymptote at tau = 1 s
  B  bias instability            -- min(sigma) / 0.664
  K  rate random walk            -- the tau^+1/2 asymptote at tau = 3 s

and converts them into the four numbers FAST-LIO's config wants. Read the
caveat this prints before pasting those in: FAST-LIO's defaults are
deliberately inflated well above true sensor noise, and dropping honest
Allan numbers in can make the filter overconfident.
"""
import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

AXES = ("gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z")


def load(bag, topic="/imu/data_raw", max_hours=None):
    db = bag
    if os.path.isdir(bag):
        hits = sorted(glob.glob(os.path.join(bag, "*.db3")))
        if not hits:
            sys.exit(f"no .db3 in {bag}")
        db = hits[0]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    row = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        sys.exit(f"no {topic} in {db}")
    q = "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp"
    args = [row[0]]
    if max_hours:
        t0 = con.execute("SELECT MIN(timestamp) FROM messages WHERE topic_id=?",
                         (row[0],)).fetchone()[0]
        q = ("SELECT timestamp, data FROM messages WHERE topic_id=? AND "
             "timestamp<=? ORDER BY timestamp")
        args.append(t0 + int(max_hours * 3600 * 1e9))

    ts, hs, vals = [], [], []
    for i, (t, blob) in enumerate(con.execute(q, args)):
        m = deserialize_message(bytes(blob), Imu)
        g, a = m.angular_velocity, m.linear_acceleration
        vals.append((g.x, g.y, g.z, a.x, a.y, a.z))
        ts.append(t)
        hs.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
        if i % 200000 == 0 and i:
            print(f"    {i:,} samples...", flush=True)
    return (np.array(ts, dtype=np.int64), np.array(hs, dtype=np.int64),
            np.array(vals, dtype=np.float64))


def allan_dev(x, dt, taus):
    """Overlapping Allan deviation of rate series x at averaging times taus."""
    theta = np.cumsum(x) * dt          # integrate rate -> angle/velocity
    theta = np.insert(theta, 0, 0.0)
    n = len(theta)
    out_t, out_s = [], []
    seen = set()
    for tau in taus:
        m = int(round(tau / dt))
        # log-spaced taus collide after rounding to whole samples; duplicates
        # produce zero-width intervals and make np.gradient divide by zero
        if m < 1 or m in seen or n - 2 * m < 1:
            continue
        seen.add(m)
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        sigma2 = np.sum(d * d) / (2.0 * (m * dt) ** 2 * len(d))
        out_t.append(m * dt)
        out_s.append(math.sqrt(sigma2))
    return np.array(out_t), np.array(out_s)


def fit_region(tau, sig, lo, hi):
    """Mean of sigma*tau^-slope over the span where the local slope fits."""
    lg_t, lg_s = np.log10(tau), np.log10(sig)
    slope = np.gradient(lg_s, lg_t)
    sel = (slope > lo) & (slope < hi)
    return sel


def characterise(tau, sig):
    res = {}
    # white noise: sigma = N / sqrt(tau) -> N = sigma * sqrt(tau)
    sel = fit_region(tau, sig, -0.65, -0.35)
    res["N"] = float(np.median(sig[sel] * np.sqrt(tau[sel]))) if sel.any() else None
    # bias instability: the floor
    i = int(np.argmin(sig))
    res["B"] = float(sig[i] / 0.664)
    res["B_tau"] = float(tau[i])
    res["B_at_edge"] = bool(i >= len(sig) - 2)   # floor not actually reached
    # rate random walk: sigma = K * sqrt(tau/3)
    sel = fit_region(tau, sig, 0.35, 0.65)
    res["K"] = float(np.median(sig[sel] / np.sqrt(tau[sel] / 3.0))) if sel.any() else None
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu/data_raw")
    ap.add_argument("--max-hours", type=float, default=None)
    ap.add_argument("--plot", default=None)
    a = ap.parse_args()

    print(f"reading {a.bag} ...", flush=True)
    ts, hs, v = load(a.bag, a.topic, a.max_hours)
    if len(v) < 10000:
        sys.exit(f"only {len(v)} samples -- far too short for Allan variance")

    span = (ts[-1] - ts[0]) / 1e9
    fs = (len(ts) - 1) / span
    dt = 1.0 / fs
    print(f"  {len(v):,} samples, {span/3600:.2f} h, {fs:.2f} Hz\n")

    # Allan variance assumes a uniform sample interval, so what matters is
    # whether the sensor missed samples -- not when the messages landed. The
    # bridge ships batches, so arrival gaps are bimodal by design and say
    # nothing. Judge dropouts on header.stamp instead.
    arr = np.diff(ts) / 1e9
    hdr = np.diff(hs) / 1e9
    print(f"arrival gaps (batched by the bridge, not meaningful): "
          f"median {np.median(arr)*1000:.2f} ms, max {arr.max()*1000:.1f} ms")
    if (hs > 0).all() and hdr.max() > 0:
        print(f"header.stamp interval: median {np.median(hdr)*1000:.3f} ms, "
              f"p99.9 {np.percentile(hdr, 99.9)*1000:.3f} ms, "
              f"max {hdr.max()*1000:.1f} ms")
        drops = int((hdr > 3 * np.median(hdr)).sum())
        print(f"  {drops} interval(s) over 3x median"
              + ("" if drops == 0 else "  <- sensor dropouts, results degraded"))
    else:
        print("  header.stamp unusable; falling back to a uniform interval")
    print()

    # tau from 2 samples up to span/9 (beyond that too few clusters to trust)
    taus = np.logspace(math.log10(2 * dt), math.log10(span / 9.0), 60)

    results = {}
    curves = {}
    for k, name in enumerate(AXES):
        t, s = allan_dev(v[:, k] - v[:, k].mean(), dt, taus)
        curves[name] = (t, s)
        results[name] = characterise(t, s)

    print(f"{'axis':9} {'N (white)':>16} {'B (bias inst)':>16} {'K (rand walk)':>16}")
    for name in AXES:
        r = results[name]
        u = "rad/s/sqrt(Hz)" if name.startswith("gyro") else "m/s^2/sqrt(Hz)"
        n = f"{r['N']:.3e}" if r["N"] else "unresolved"
        b = f"{r['B']:.3e}" if r["B"] else "unresolved"
        kk = f"{r['K']:.3e}" if r["K"] else "unresolved"
        flag = " *" if r["B_at_edge"] else ""
        print(f"{name:9} {n:>16} {b:>16}{flag} {kk:>16}   [{u}]")
    if any(results[n]["B_at_edge"] for n in AXES):
        print("\n * the Allan curve was still falling at the longest tau, so the")
        print("   bias-instability floor was never reached. That number is an")
        print("   upper bound only -- record for longer.")

    gN = np.mean([results[n]["N"] for n in AXES[:3] if results[n]["N"]])
    aN = np.mean([results[n]["N"] for n in AXES[3:] if results[n]["N"]])
    gK = [results[n]["K"] for n in AXES[:3] if results[n]["K"]]
    aK = [results[n]["K"] for n in AXES[3:] if results[n]["K"]]

    print("\n" + "=" * 62)
    print("FAST-LIO config equivalents (variances, axis-averaged):")
    print(f"  gyr_cov:   {gN**2:.3e}")
    print(f"  acc_cov:   {aN**2:.3e}")
    if gK:
        print(f"  b_gyr_cov: {np.mean(gK)**2:.3e}")
    if aK:
        print(f"  b_acc_cov: {np.mean(aK)**2:.3e}")
    print("=" * 62)
    print("""
CAVEAT, read before pasting these in. The config currently runs
gyr_cov/acc_cov 0.1 and b_*_cov 1e-4, which are FAST-LIO's defaults and
sit orders of magnitude above any real MEMS sensor's noise. That is
deliberate: the process noise absorbs un-modelled error -- vibration,
deskew residual, extrinsic error -- not just sensor noise. Substituting
honest Allan numbers makes the filter trust the IMU far more, which can
help or can diverge. Treat this as an A/B against a known-good bag, not
as a correction.""")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
        for k, name in enumerate(AXES):
            t, s = curves[name]
            axes[0 if k < 3 else 1].loglog(t, s, label=name)
        for ax, ttl, u in ((axes[0], "gyroscope", "rad/s"),
                           (axes[1], "accelerometer", "m/s$^2$")):
            ax.set_xlabel(r"$\tau$ (s)")
            ax.set_ylabel(rf"$\sigma(\tau)$ ({u})")
            ax.set_title(ttl)
            ax.grid(True, which="both", alpha=.3)
            ax.legend()
        plt.tight_layout()
        plt.savefig(a.plot, dpi=120)
        print(f"\nwrote {a.plot}")


if __name__ == "__main__":
    main()
