#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from collections import namedtuple
from math import cos
from math import pi

Motor = namedtuple("Motor", ["pin0", "pin1", "pwm_pin"])
motor_left = Motor(pin0 = 15, pin1 = 13, pwm_pin = 11)
motor_right = Motor(pin0 = 19, pin1 = 21, pwm_pin = 23)
motors = (motor_left, motor_right)

def clamp(minimum, x, maximum):
    if x < minimum:
        return minimum
    elif x > maximum:
        return maximum
    else:
        return x

def normclamp(x):
    return clamp(-1, x, 1)

def set_speed(motor_id, speed):
    rospy.loginfo(format('Motor {} spinning at {}'.format(motor_id, speed)))

def callback(data):
    speed = data.linear.z
    direction = clamp(-1, data.angular.y, 1) * pi/2 + pi/2
    set_speed(0, normclamp(2*cos(direction)+1)*speed)
    set_speed(1, normclamp(-2*cos(direction)+1)*speed)

def listener():
    rospy.init_node('cassiopeia_motors')

    rospy.Subscriber('cassiopeia/motors_control', Twist, callback)
    
    rospy.spin()

if __name__=='__main__':
    listener()