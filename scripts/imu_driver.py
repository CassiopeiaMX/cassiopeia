#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Quaternion
from mpu6050 import mpu6050

hz = 10
delta_t = 1 / hz

angle = Quaternion(x=0, y=0, z=0)
noise_gate_level = 10
noise_gate_level = noise_gate_level / hz

sensor = mpu6050(0x68)

calibration_time = 5


def calibrate():
    rospy.loginfo("Calibrating for " + str(calibration_time) + " seconds. Don't move sensor.")
    offset = Quaternion()
    i = 0
    n = calibration_time / delta_t
    rate = rospy.Rate(hz)
    while i < n:
        offset = add_quaternions(offset, get_angular_vel())
        i = i + 1
        rate.sleep()
    offset = const_times_quaternion(-1 / n, offset)
    rospy.loginfo("Calibrated!")
    return offset


def const_times_quaternion(c, q):
    return Quaternion(x=c * q.x, y=c * q.y, z=c * q.z)


def add_quaternions(q1, q2):
    return Quaternion(x=q1.x + q2.x, y=q1.y + q2.y, z=q1.z + q2.z)


def noise_gate_filter(q):
    if abs(q.x) < noise_gate_level:
        q.x = 0
    if abs(q.y) < noise_gate_level:
        q.y = 0
    if abs(q.z) < noise_gate_level:
        q.z = 0
    return q


def get_angular_vel():
    data = sensor.get_gyro_data()
    return Quaternion(x=data['x'], y=data['y'], z=data['z'])


def talker():
    global angle
    pub = rospy.Publisher('cassiopeia/camera_control', Quaternion, queue_size=10)
    rospy.init_node('imu_driver', anonymous=False)
    rate = rospy.Rate(hz)
    offset = calibrate()
    while not rospy.is_shutdown():
        angular_vel = get_angular_vel()
        angular_vel = add_quaternions(angular_vel, offset)
        angular_vel = noise_gate_filter(angular_vel)
        delta_angle = const_times_quaternion(delta_t, angular_vel)
        angle = add_quaternions(angle, delta_angle)
        pub.publish(angle)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
