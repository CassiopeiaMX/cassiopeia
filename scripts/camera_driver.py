#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import sys
from os import path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from camera import Camera


def run():
    pub = rospy.Publisher('cassiopeia/image_raw/compressed', CompressedImage, queue_size=1)
    rospy.init_node('camera_driver', anonymous=False)
    rate = rospy.Rate(60)

    camera = Camera()

    while True:
        image = camera.value
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass