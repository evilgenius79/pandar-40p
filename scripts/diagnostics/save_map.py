#!/usr/bin/env python3
"""
Accumulate /cloud_registered into a single PCD.

Usage:
    python3 ~/save_map.py [voxel_size] [output_path]

Defaults: voxel 0.05 m, output ~/map.pcd
Press Ctrl+C when the bag finishes; the file is written on exit.
"""
import sys
import signal
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

VOXEL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
OUT = sys.argv[2] if len(sys.argv) > 2 else '/home/lidar/map.pcd'

chunks = []
nscans = [0]


def write_pcd():
    if not chunks:
        print('\nNo scans received - nothing to write.')
        return
    print('\nconcatenating...')
    pts = np.concatenate(chunks, axis=0)
    print(f'  raw points: {len(pts):,}')

    print(f'voxel downsampling at {VOXEL} m ...')
    keys = np.floor(pts / VOXEL).astype(np.int64)
    # unique rows
    _, idx = np.unique(keys, axis=0, return_index=True)
    pts = pts[np.sort(idx)]
    print(f'  kept points: {len(pts):,}')

    pts = pts.astype(np.float32)
    n = len(pts)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    with open(OUT, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.tobytes())
    print(f'wrote {OUT}  ({n:,} points)')


def cb(msg):
    arr = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
    arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
    good = np.isfinite(arr).all(axis=1)
    chunks.append(arr[good])
    nscans[0] += 1
    if nscans[0] % 50 == 0:
        total = sum(len(c) for c in chunks)
        print(f'  {nscans[0]} scans, {total:,} points')


def main():
    rclpy.init()
    node = rclpy.create_node('save_map')
    node.create_subscription(PointCloud2, '/cloud_registered', cb, 50)
    print(f'accumulating /cloud_registered  (voxel {VOXEL} m)')
    print('Ctrl+C when the bag finishes.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # noqa: BLE001
        # rclpy raises ExternalShutdownException when SIGINT arrives, which
        # it handles itself rather than letting Python raise KeyboardInterrupt.
        # The PCD still gets written by the finally block below, but the
        # traceback made a successful run look like a crash. Swallow it by
        # name without importing a version-specific symbol.
        if type(exc).__name__ != 'ExternalShutdownException':
            raise
    finally:
        write_pcd()


if __name__ == '__main__':
    main()
