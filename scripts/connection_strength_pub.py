#!/usr/bin/env python

from subprocess import Popen, PIPE
import rospy
from std_msgs.msg import Int32

def talker():
    connection_strength_pub = rospy.Publisher('cassiopeia/connection_strength', Int32, queue_size=10)
    rospy.init_node('connection_strength_pub', anonymous=False)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        process = Popen(["nmcli", "dev", "wifi"], stdout=PIPE)
        stdout = process.communicate()
        a = stdout[0].split()
        connection_strength = int(a[14])
        msg = Int32(data=connection_strength)
        connection_strength_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
