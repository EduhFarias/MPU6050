from mpu6050 import mpu6050

sensor = mpu6050(0x68)

a_x, a_y, a_z = sensor.get_accel_data()

print('X: {} | Y: {} | Z: {}'.format(a_x, a_y, a_z))
