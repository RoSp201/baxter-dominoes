#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge

def camera_feed_face():
	rospy.init_node("test_img_pub")
	img_pub = rospy.Publisher("/testimg", Image, queue_size=10)
	test = cv2.imread("/home/cc/ee106a/fa16/class/ee106a-abo/ee106a-baxter-project/motion_planning/motion/src/follow/src/img.png")
	while 1:
		img_pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(test, encoding="bgr8"))
	rospy.spin()

if __name__ == "__main__":
    camera_feed_face()
