#!/usr/bin/env python

import curses

import rospy
from geometry_msgs.msg import Quaternion

angle = Quaternion()
angular_velocity = 50
hz = 50
delta_t = 1 / hz


def const_times_quaternion(c, q):
    return Quaternion(x=c * q.x, y=c * q.y, z=c * q.z)


def add_quaternions(q1, q2):
    return Quaternion(x=q1.x + q2.x, y=q1.y + q2.y, z=q1.z + q2.z)


def get_input(win):
    try:
        key = win.getkey()
        return key
    except Exception as e:
        return ' '


def talker(win):
    global angle
    pub = rospy.Publisher('cassiopeia/camera_control', Quaternion, queue_size=10)
    rospy.init_node('keyboard_camera_control', anonymous=False)
    rate = rospy.Rate(hz)
    win.nodelay(True)
    while not rospy.is_shutdown():
        yaw = 0
        pitch = 0
        c = get_input(win)
        if c == 'a':
            yaw = yaw - angular_velocity
        if c == 'd':
            yaw = yaw + angular_velocity
        if c == 'w':
            pitch = pitch + angular_velocity
        if c == 's':
            pitch = pitch - angular_velocity
        delta_angle = Quaternion(x=0, y=pitch, z=yaw)
        delta_angle = const_times_quaternion(delta_t, delta_angle)
        angle = add_quaternions(angle, delta_angle)
        pub.publish(angle)
        rate.sleep()


if __name__ == '__main__':
    try:
        curses.wrapper(talker)
    except rospy.ROSInterruptException:
        pass
