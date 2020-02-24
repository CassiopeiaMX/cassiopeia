#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion
import sys
from os import path

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from servo import Servo

yaw_servo = Servo(32, 2.9, 11.9, 180)
pitch_servo = Servo(33, 5.3, 12.3, 180)


def callback(data):
    yaw_deg = data.z
    pitch_deg = data.y
    yaw_servo.rotate(yaw_deg)
    pitch_servo.rotate(pitch_deg)


def listener():
    rospy.Subscriber('cassiopeia/camera_control', Quaternion, callback, queue_size=1)

    rospy.init_node('camera_control')
    rospy.spin()


if __name__ == '__main__':
    listener()
