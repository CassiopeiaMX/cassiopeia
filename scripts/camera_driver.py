#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import sys
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from camera import Camera


def main():
    pub = rospy.Publisher('cassiopeia/image_raw/compressed', ImageCompressed)
    rospy.init_node('cassiopeia/camera_driver', anonymous=False)

    camera = Camera()

    while True:
        image = camera.value
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        pub.publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass