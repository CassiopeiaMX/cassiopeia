#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import imu


def talker():
    pub = rospy.Publisher('cassiopeia/imu/', Twist, queue_size=10)
    rospy.init_node('imu_driver', anonymous=False)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        angular_acceleration = imu.get_gyro()
        linear_acceleration = imu.get_acceleration()
        linear = Vector3(x=linear_acceleration[0],
                         y=linear_acceleration[1],
                         z=linear_acceleration[2])
        angular = Vector3(x=angular_acceleration[0],
                          y=angular_acceleration[1],
                          z=angular_acceleration[2])
        twist = Twist(linear=linear, angular=angular)
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
