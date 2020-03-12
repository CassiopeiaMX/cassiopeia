#!/usr/bin/env python3

import board
import busio
from adafruit_servokit import ServoKit
import rospy
from cassiopeia.msg import CameraTwist
from cassiopeia.msg import CameraPosition

pitch_servo_index = 1
yaw_servo_index = 0

pitch_min = 40
pitch_max = 170
yaw_min = 0
yaw_max = 180

twist_speed = 90.0  # degrees per second

pitch_servo = None
yaw_servo = None

rate = 40.0
delta_t = 1.0 / rate

pitch_inertia = 0
yaw_inertia = 0

def clamp(val, minimum, maximum):
    if val > maximum:
        return maximum
    if val < minimum:
        return minimum
    return val


def setup():
    global pitch_servo
    global yaw_servo
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    kit = ServoKit(channels=16, i2c=i2c_bus)
    pitch_servo = kit.servo[pitch_servo_index]
    yaw_servo = kit.servo[yaw_servo_index]
    update_servos((yaw_max+yaw_min)/2, (pitch_max+pitch_min)/2)


def absolute_callback(data):
    global pitch_inertia
    global yaw_inertia
    pitch_inertia = 0
    yaw_inertia = 0
    update_servos(yaw=data.yaw, pitch=data.y)


def twist_callback(data):
    global pitch_inertia
    global yaw_inertia
    pitch_inertia = twist_speed * data.pitch
    yaw_inertia = twist_speed * data.yaw


def update_servos(yaw, pitch):
    global pitch_servo
    global yaw_servo
    yaw_servo.angle = clamp(yaw, yaw_min, yaw_max)
    pitch_servo.angle = clamp(pitch, pitch_min, pitch_max)


def listener():
    rospy.Subscriber('cassiopeia/control/camera/absolute', CameraPosition, absolute_callback, queue_size=1)
    rospy.Subscriber('cassiopeia/control/camera/twist', CameraTwist, twist_callback, queue_size=1)
    rospy.init_node('camera_controller')
    ros_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        yaw = yaw_servo.angle + yaw_inertia * delta_t
        pitch = pitch_servo.angle + pitch_inertia * delta_t
        update_servos(yaw=yaw, pitch=pitch)
        ros_rate.sleep()


if __name__ == '__main__':
    setup()
    listener()
