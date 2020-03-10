#!/usr/bin/env python3


import imu
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def timer_callback(event):
    imu.move_angle()


def talker():
    pub = rospy.Publisher('cassiopeia/imu/', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=False)
    rate = rospy.Rate(5)
    rospy.Timer(rospy.Duration(imu.get_delta_t()), timer_callback)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        angular_speed = imu.get_gyro()
        linear_acceleration = imu.get_acceleration()
        linear = Vector3(x=linear_acceleration[0],
                         y=linear_acceleration[1],
                         z=linear_acceleration[2])
        angular = Vector3(x=angular_speed[0],
                          y=angular_speed[1],
                          z=angular_speed[2])
        angle = imu.get_angle()
        # TODO: quaternion is technically wrong like this. w should be roll. But using rotations on axis as base
        orientation = Quaternion(x=angle[0],
                                 y=angle[1],
                                 z=angle[2])
        imu_msg = Imu(linear_acceleration=linear, angular_velocity=angular, orientation=orientation, header=h)
        pub.publish(imu_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
