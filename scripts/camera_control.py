#!/usr/bin/env python3

import board
import busio
from adafruit_servokit import ServoKit
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

pitch_servo_index = 1
yaw_servo_index = 0

pitch_min = 40
pitch_max = 170
yaw_min = 0
yaw_max = 180

twist_speed = 5.0

pitch_servo = None
yaw_servo = None


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
    update_servos(90, 90)


def absolute_callback(data):
    update_servos(yaw=data.z, pitch=data.y)


def twist_callback(data):
    update_servos(yaw=yaw_servo.angle+twist_speed*data.angular.z, pitch=pitch_servo.angle+twist_speed*data.linear.x)


def joy_callback(data):
    pitch = pitch_servo.angle
    pitch = pitch + twist_speed * (data.buttons[4] - data.buttons[5])
    yaw = yaw_servo.angle
    yaw = yaw + twist_speed * (data.buttons[6] - data.buttons[7])
    update_servos(yaw=yaw, pitch=pitch)


def update_servos(yaw, pitch):
    global pitch_servo
    global yaw_servo
    print(yaw)
    yaw_servo.angle = clamp(yaw, yaw_min, yaw_max)
    pitch_servo.angle = clamp(pitch, pitch_min, pitch_max)


def listener():
    rospy.Subscriber('cassiopeia/camera_control/absolute', Quaternion, absolute_callback, queue_size=1)
    rospy.Subscriber('cassiopeia/camera_control/twist', Twist, twist_callback, queue_size=1)
    rospy.Subscriber('cassiopeia/input/joy', Joy, joy_callback, queue_size=1)
    rospy.init_node('camera_control')
    rospy.spin()


if __name__ == '__main__':
    setup()
    listener()
