#!/usr/bin/env python

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import Image

def callback(message):
	# Just displays result
	img = cv_bridge.CvBridge().imgmsg_to_cv2(message, desired_encoding="passthrough")
	cv2.imshow('img', img)
	cv2.waitKey(1)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/baxter_image", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()