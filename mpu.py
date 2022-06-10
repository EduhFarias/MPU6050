from mpu6050 import mpu6050
import numpy as np
import json
from time import sleep

sensor = mpu6050(0x68)

x, y, z = [], [], []
i = 0

while(i < 50):
  accel_data = sensor.get_accel_data()
  ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
  print('X: {} | Y: {} | Z: {}'.format(ax, ay, az))
  x.append(ax)
  y.append(ay)
  z.append(az)
  i += 0.01
  sleep(0.01)

with open('tst1.json', 'w') as outp:  # Overwrites any existing file.
  json.dump({'x': x, 'y': y, 'z': z}, outp)
