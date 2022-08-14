from mpu6050 import mpu6050
import numpy as np
import json
from time import sleep

sensor = mpu6050(0x68)

xa, ya, za = [], [], []
xg, yg, zg = [], [], []
i = 0

while (i < 20):
    accel_data = sensor.get_accel_data()
    gyro = sensor.get_gyro_data()
    ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
    gx. gy, gz = gyro['x'], gyro['z'], gyro['z']
    print('Accel -> X: {} | Y: {} | Z: {}'.format(ax, ay, az))
    print('Gyro  -> X: {} | Y: {} | Z: {}'.format(gx, gy, gz))
    xa.append(ax)
    ya.append(ay)
    za.append(az)
    xg.append(gx)
    yg.append(gy)
    zg.append(gz)
    i += 0.5
    sleep(0.5)

with open('mpu.json', 'w') as outp:  # Overwrites any existing file.
    json.dump({'accel': {'x': xa, 'y': ya, 'z': za},
              'gyro': {'x': xg, 'y': yg, 'z': zg}}, outp)
