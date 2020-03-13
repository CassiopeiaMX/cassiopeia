#!/usr/bin/env python3
import time

import rospy
import serial
from cassiopeia.msg import Trit
from cassiopeia.msg import Vector2
from serial.serialutil import SerialException

ser = serial.Serial()
ser.port = '/dev/ttyACM0'
ser.baudrate = 9600

ser_update_time = 50 / 1000
past_code = 99

arm = 0
shovel = 0
dir_code = 9
code = 99


def trit_to_int(data):
    val = 0
    if data.magnitude:
        val = 1
    if data.negative:
        val = val * -1
    return val

def sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0


def dir_callback(data):
    global dir_code

    x = data.x
    y = data.y
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

    dir_code = mc

    update_code()


def shovel_callback(data):
    global shovel
    shovel = trit_to_int(data)
    update_code()


def arm_callback(data):
    print("aaa")
    global arm
    arm = trit_to_int(data)
    update_code()


def update_code():
    arm_combinations = [
        [1, 1],
        [1, 0],
        [1, -1],
        [-1, 0],
        [-1, -1],
        [0, -1],
        [-1, 1],
        [0, 1],
        [0, 0]
    ]

    comb = [arm, shovel]
    arm_code = arm_combinations.index(comb) + 1

    code = dir_code * 10 + arm_code
    print(code)
    send_code(code)


def send_code(code):
    global past_code

    tx = "{} \n".format(code)
    tx = tx.encode(encoding='ascii')

    if ser.is_open:
        rospy.logdebug("Writing {} to {}".format(tx, ser.port))
        ser.write(tx)
        past_code = code


def listener():
    connected = False
    reconnect_time = 5
    while not connected:
        try:
            rospy.logwarn("Opening port {}".format(ser.port))
            ser.open()
            rospy.logwarn("Done!")
            connected = True
        except SerialException:
            rospy.logwarn("Could not open {}. Trying again in {} seconds.".format(ser.port, reconnect_time))
            time.sleep(reconnect_time)
    rospy.Subscriber('cassiopeia/control/direction', Vector2, dir_callback)
    rospy.Subscriber('cassiopeia/control/arm', Trit, arm_callback)
    rospy.Subscriber('cassiopeia/control/shovel', Trit, shovel_callback)
    rospy.init_node('cassiopeia_motors')
    rate = rospy.Rate(1 / ser_update_time)
    while not rospy.is_shutdown():
        rate.sleep()
        send_code(past_code)


if __name__ == '__main__':
    listener()
