#!/usr/bin/env python

from collections import namedtuple
from math import cos
from math import pi

import Jetson.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

Motor = namedtuple("Motor", ["pin0", "pin1", "pwm_pin"])
motor_left = Motor(pin0=15, pin1=13, pwm_pin=11)
motor_right = Motor(pin0=19, pin1=21, pwm_pin=23)


def clamp(minimum, x, maximum):
    if x < minimum:
        return minimum
    elif x > maximum:
        return maximum
    else:
        return x


def normal_clamp(x):
    return clamp(-1, x, 1)


def outputs(pins, values):
    for i in range(0, len(pins)):
        GPIO.output(pins[i], values[i])


def set_speed(motor, speed):
    GPIO.output(motor.pwm_pin, 1)
    if -1 <= speed < -0.5:
        GPIO.output(motor.pin1, 0)
        GPIO.output(motor.pin0, 1)
    elif -0.5 <= speed <= 0:
        GPIO.output(motor.pin1, 0)
        GPIO.output(motor.pin0, 0)
    elif 0 < speed <= 1:
        GPIO.output(motor.pin1, 1)
        GPIO.output(motor.pin0, 0)


def callback(data):
    if data.linear.x > 0:
        speed = 1
    elif data.linear.x < 0:
        speed = -1
    else:
        speed = 0
    direction = clamp(-1, data.angular.z, 1) * pi / 2 + pi / 2
    set_speed(motor_left, normal_clamp(2 * cos(direction) + 1) * speed)
    set_speed(motor_right, normal_clamp(-2 * cos(direction) + 1) * speed)


def listener():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(motor_left, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(motor_right, GPIO.OUT, initial=GPIO.LOW)

    rospy.init_node('cassiopeia_motors')

    rospy.Subscriber('cassiopeia/motors_control', Twist, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
    GPIO.cleanup()
