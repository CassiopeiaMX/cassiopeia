#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('cassiopeia_motors')

    rospy.Subscriber('motors_control', Twist)
    
    rospy.spin()

if __name__=='__main__':
    listener()