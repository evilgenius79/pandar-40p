import math
import rclpy
from sensor_msgs.msg import Imu

rclpy.init()
n = rclpy.create_node('imu_units')

N = 600
gmax = [0.0]
anorms = []
count = [0]


def cb(msg):
    g = msg.angular_velocity
    a = msg.linear_acceleration
    gmag = math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)
    if gmag > gmax[0]:
        gmax[0] = gmag
    anorms.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
    count[0] += 1
    if count[0] % 100 == 0:
        print(f'  ...{count[0]} samples, peak |gyro| so far = {gmax[0]:.3f}')
    if count[0] >= N:
        amean = sum(anorms) / len(anorms)
        print('')
        print('================ RESULT ================')
        print(f'samples            : {count[0]}')
        print(f'peak |gyro|        : {gmax[0]:.3f}')
        print(f'mean |accel|       : {amean:.3f}')
        print('')
        if gmax[0] > 10.0:
            print('>>> GYRO LOOKS LIKE deg/s  (peak > 10)')
            print('>>> FAST-LIO expects rad/s. This is the bug.')
        elif gmax[0] > 0.05:
            print('>>> GYRO LOOKS LIKE rad/s  (peak in a sane range)')
            print('>>> Units are fine; suspect the extrinsic instead.')
        else:
            print('>>> GYRO NEAR ZERO - was the rig moving/rotating?')
            print('>>> Re-run during the moving part of the bag.')
        print('')
        if 9.0 < amean < 10.6:
            print('accel: m/s^2, gravity present. Correct.')
        elif 0.9 < amean < 1.1:
            print('accel: in g units. FAST-LIO auto-scales, so not the whip cause.')
        else:
            print(f'accel: unexpected magnitude {amean:.3f} - worth a look.')
        print('========================================')
        rclpy.shutdown()


n.create_subscription(Imu, '/imu/data_raw', cb, 200)
print('collecting IMU samples from /imu/data_raw ...')
print('(play the bag; let it reach the MOVING section)')
rclpy.spin(n)
