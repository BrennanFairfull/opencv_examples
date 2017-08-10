#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import roslib
import sys

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def talker():

    # define publisher and topic name
    image_pub = rospy.Publisher("cam_feed", Image, queue_size=4)

    # initialize node
    rospy.init_node('camera_sender', anonymous=True)
    
    #define cvbridge handler
    bridge = CvBridge();

    #define capture
    cap = cv2.VideoCapture(0)

    #set node frequency
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # Try to publish the image
        try:
	    cvImg = cap.read()
            image_pub.publish(bridge.cv2_to_imgmsg(cvImg, "bgr8"))
            rospy.loginfo("Image sent!")
        except CvBridgeError as e:
            print(e)
            cap.release()

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
