#!/usr/bin/env python3

import serial

import rospy
from sensor_msgs.msg import Joy

ser = serial.Serial('/dev/ttyACM0')
ser.baudrate = 9600

def sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0


def callback(data):
    x = data.axes[0]
    y = data.axes[1]
    d = 0.2
    mc = 9
    if -d < x < d and -d < y < d:
        mc = 9
    if -d < x < d and y > d:
        mc = 1
    if -d < x < d and y < -d:
        mc = 5
    if x > d and -d < y < d:
        mc = 3
    if x < -d and -d < y < d:
        mc = 7
    if x < -d and y > d:
        mc = 8
    if x > d and y > d:
        mc = 2
    if x < -d and y < -d:
        mc = 6
    if x > d and y < -d:
        mc = 4

    arm_throttle = data.buttons[0] - data.buttons[1]
    shovel_throttle = data.buttons[2] - data.buttons[3]

    arm_combinations = [
        [ 1, 1],
        [ 1, 0],
        [ 1,-1],
        [-1, 0],
        [-1,-1],
        [ 0,-1],
        [-1, 1],
        [ 0, 1],
        [ 0, 0]
    ]

    comb = [arm_throttle, shovel_throttle]
    arm_code = arm_combinations.index(comb) + 1

    code = mc * 10 + arm_code

    tx = "{} \n".format(code)
    tx = tx.encode(encoding='ascii')

    ser.write(tx)


def listener():
    rospy.init_node('cassiopeia_motors')
    rospy.Subscriber('cassiopeia/input/joy', Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
