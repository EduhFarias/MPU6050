import rclpy
from rclpy.node import Node
from math import sqrt
import serial
import struct
import json
from mpu6050 import mpu6050

from geometry_msgs.msg import Twist

MOTOR1 = b'\xf5'  # address serial motor 1
MOTOR2 = b'\xfa'  # address serial motor 2
MOTOR3 = b'\xff'  # address serial motor 3

FOWARD = b'\x66'  # turn motor foward
BACKWARD = b'\x62' 	# turn motor backward
STOP = b'\x73'	      	# stop motor


def serial_motor(vel1, vel2, vel3, serial):
    serial.write(MOTOR1)
    if vel1 < 0:
        serial.write(struct.pack("B", vel1 * -1))
        serial.write(FOWARD)
    if vel1 > 0:
        serial.write(struct.pack("B", vel1))
        serial.write(BACKWARD)
    if vel1 == 0:
        serial.write(struct.pack("B", 0))
        serial.write(STOP)

    serial.write(MOTOR2)
    if vel2 < 0:
        serial.write(struct.pack("B", vel2 * -1))
        serial.write(FOWARD)
    if vel2 > 0:
        serial.write(struct.pack("B", vel2))
        serial.write(BACKWARD)
    if vel2 == 0:
        serial.write(struct.pack("B", 0))
        serial.write(STOP)

    serial.write(MOTOR3)
    if vel3 < 0:
        serial.write(struct.pack("B", vel3 * -1))
        serial.write(FOWARD)
    if vel3 > 0:
        serial.write(struct.pack("B", vel3))
        serial.write(BACKWARD)
    if vel3 == 0:
        serial.write(struct.pack("B", 0))
        serial.write(STOP)


class OmninoDriverNode(Node):
    def __init__(self):
        super().__init__('omnino_driver')
        self.ser = serial.Serial('/dev/ttyS0', 9600)

        self.declare_parameter("wheel_r", 0.02)
        self.declare_parameter("robot_d", 0.07)

        self.r = self.get_parameter(
            "wheel_r").get_parameter_value().double_value
        self.d = self.get_parameter(
            "robot_d").get_parameter_value().double_value

        self.sub_ = self.create_subscription(
            Twist, 'cmd_vel', self.driver_callback, 10)

    def driver_callback(self, cmd_vel):
        vx = cmd_vel.linear.x
        vy = cmd_vel.linear.y
        w = cmd_vel.angular.z

        vel_1 = 1/self.r * (vx - self.d*w)
        vel_2 = 1/self.r * (-1/2*vx - sqrt(3)/2*vy - self.d*w)
        vel_3 = 1/self.r * (-1/2*vx + sqrt(3)/2*vy - self.d*w)

        serial_motor(vel_1, vel_2, vel_3, self.ser)


def main(args=None):
    ser = serial.Serial('/dev/ttyS0', 9600)
    sensor = mpu6050(0x68)
    vx, vy, vz = 0, 1, 0
    r = 0.02
    d = 0.07
    vel_1 = 1/r * (vx - d*w)
    vel_2 = 1/r * (-1/2*vx - sqrt(3)/2*vy - d*w)
    vel_3 = 1/r * (-1/2*vx + sqrt(3)/2*vy - d*w)
    x, y, z = [], [], []
    i = 0
    serial_motor(vel_1, vel_2, vel_3, ser)
    while(i < 50):
    accel_data = sensor.get_accel_data()
    ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
    x.append(ax)
    y.append(ay)
    z.append(az)
    i += 0.5
    sleep(0.5)

    with open('omnino.json', 'w') as outp:  # Overwrites any existing file.
    json.dump({'x': x, 'y': y, 'z': z}, outp)
    # rclpy.init(args=args)
    # omnino_driver_node = OmninoDriverNode()
    # rclpy.spin(omnino_driver_node)
    # omnino_driver_node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
