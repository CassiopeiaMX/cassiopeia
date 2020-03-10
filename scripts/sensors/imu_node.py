#!/usr/bin/env python3


import imu
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


def talker():
    pub = rospy.Publisher('cassiopeia/imu/', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=False)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        angular_acceleration = imu.get_gyro()
        linear_acceleration = imu.get_acceleration()
        linear = Vector3(x=linear_acceleration[0],
                         y=linear_acceleration[1],
                         z=linear_acceleration[2])
        angular = Vector3(x=angular_acceleration[0],
                             y=angular_acceleration[1],
                             z=angular_acceleration[2])
        imu_msg = Imu(linear_acceleration=linear, angular_velocity=angular, header=h)
        pub.publish(imu_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
