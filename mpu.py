from mpu6050 import mpu6050
import numpy as np
import json

sensor = mpu6050(0x68)

x, y, z = [], [], []
i = 0

while(i < 1):
  accel_data = sensor.get_accel_data()
  ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
  print('X: {} | Y: {} | Z: {}'.format(ax, ay, az))
  x.push(ax)
  y.push(ay)
  z.push(az)
  i += 0.01

with open('tst1.json', 'w') as outp:  # Overwrites any existing file.
  json.dump({'x': x, 'y': y, 'z': z}, outp)
