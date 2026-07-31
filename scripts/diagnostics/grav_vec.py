import math
import rclpy
from sensor_msgs.msg import Imu

rclpy.init()
n = rclpy.create_node('grav_vec')

N = 400
sx = [0.0]
sy = [0.0]
sz = [0.0]
count = [0]


def cb(msg):
    a = msg.linear_acceleration
    sx[0] += a.x
    sy[0] += a.y
    sz[0] += a.z
    count[0] += 1
    if count[0] >= N:
        x = sx[0] / N
        y = sy[0] / N
        z = sz[0] / N
        mag = math.sqrt(x * x + y * y + z * z)
        print('')
        print('========= GRAVITY IN IMU FRAME =========')
        print(f'  ax = {x:+7.3f}')
        print(f'  ay = {y:+7.3f}')
        print(f'  az = {z:+7.3f}')
        print(f'  |a| = {mag:6.3f}')
        print('')
        # unit vector
        ux, uy, uz = x / mag, y / mag, z / mag
        print(f'  unit = [{ux:+.4f}, {uy:+.4f}, {uz:+.4f}]')
        print('')
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(uz)))))
        print(f'  tilt of IMU Z from vertical: {tilt:.1f} deg')
        if tilt < 10:
            print('  -> IMU is essentially FLAT (Z near vertical)')
        elif 35 < tilt < 55:
            print('  -> IMU is tilted ~45 deg (mounted on the tilted plate?)')
        else:
            print('  -> IMU tilt is neither flat nor 45 deg')
        print('========================================')
        rclpy.shutdown()


n.create_subscription(Imu, '/imu/data_raw', cb, 200)
print('averaging 400 samples - run during the STATIONARY opening of the bag')
rclpy.spin(n)
