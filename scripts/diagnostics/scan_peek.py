import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

rclpy.init()
n = rclpy.create_node('scan_peek')
count = [0]


def cb(msg):
    pts = list(point_cloud2.read_points(msg, field_names=['x', 'y', 'z']))
    if not pts:
        print('EMPTY CLOUD')
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    print(f'n={len(pts):6d}  '
          f'x[{min(xs):7.2f},{max(xs):7.2f}]  '
          f'y[{min(ys):7.2f},{max(ys):7.2f}]  '
          f'z[{min(zs):7.2f},{max(zs):7.2f}]')
    count[0] += 1
    if count[0] >= 5:
        rclpy.shutdown()


n.create_subscription(PointCloud2, '/cloud_registered', cb, 10)
print('waiting for /cloud_registered ...')
rclpy.spin(n)
