#!/usr/bin/env python3

from collections import namedtuple
import rospy
from geometry_msgs.msg import Twist
import busio
import board
import digitalio
from adafruit_pca9685 import PCA9685

Motor = namedtuple("Motor", ["pin0", "pin1", "pwm_channel"])
arm_motor = None
shovel_motor = None

pca = None


def spin_motor(motor, direction):
    if direction > 0:
        motor.pin0.value = True
        motor.pin1.value = False
    elif direction < 0:
        motor.pin0.value = False
        motor.pin1.value = True
    else:
        motor.pin0.value = False
        motor.pin1.value = False


def setup_motor_pins(motor):
    motor.pin0.direction = digitalio.Direction.OUTPUT
    motor.pin1.direction = digitalio.Direction.OUTPUT


def motors_setup():
    global arm_motor
    global shovel_motor
    arm_motor = Motor(pin0=digitalio.DigitalInOut(board.D5), pin1=digitalio.DigitalInOut(board.D6), pwm_channel=14)
    shovel_motor = Motor(pin0=digitalio.DigitalInOut(board.D13), pin1=digitalio.DigitalInOut(board.D19), pwm_channel=15)


def pca_setup():
    global pca
    i2c_bus = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 60


def setup():
    global pca
    global arm_motor
    global shovel_motor
    i2c_bus = busio.I2C(board.SCl, board.SDA)


def callback(data):
    arm_twist = data.linear.x
    shovel_twist = data.angular.z
    twist_motor(arm_motor, arm_twist)
    twist_motor(shovel_motor, shovel_twist)


def listener():
    rospy.Subscriber('cassiopeia/arm_control/twist', Twist, callback, queue_size=1)
    rospy.init_node('arm_control')
    rospy.spin()


if __name__ == '__main__':
    setup()
    listener()
