from mpu6050 import mpu6050
import numpy as np
import json

sensor = mpu6050(0x68)

x, y, z = [], [], []
i = 0

while(i < 1):
  a_x, a_y, a_z = sensor.get_accel_data()
  print('X: {} | Y: {} | Z: {}'.format(a_x, a_y, a_z))
  i += 0.01

with open('tst1.json', 'w') as outp:  # Overwrites any existing file.
  json.dump({'x': x, 'y': y, 'z': z}, outp)
