#!/usr/bin/env python

from math import atan
from math import degrees

import rospy
from sensor_msgs.msg import Joy
from motor import Motor


#TODO set motor pins
motor_left = Motor(pin1=x, pin2=x, pwm_channel=x)
motor_right = Motor(pin1=x, pin2=x, pwm_channel=x)


def r_of_angle(a):
    if 0 <= a < 90:
        return (2/90)*a-1
    if 90 <= a <= 180:
        return 1
    if 180 < a < 270:
        return (-2/90)*a+1
    if 270 <= a <= 360:
        return -1


def l_of_angle(a):
    if 0 <= a <= 90:
        return 1
    if 90 < a < 180:
        return (-2/90)*a+1
    if 180 <= a <= 270:
        return -1
    if 270 <= a <= 360:
        return (2/90)*a-1


def callback(data):
    x = data.axis[0]
    y = data.axis[1]
    r = (x**2 + y**2)**(1/2)
    a = degrees(atan(y/x))

    motor_left.throttle = r * l_of_angle(a)
    motor_right.throttle = r * r_of_angle(a)


def listener():
    rospy.init_node('cassiopeia_motors')

    rospy.Subscriber('cassiopeia/motors_control', Joy, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
    GPIO.cleanup()
