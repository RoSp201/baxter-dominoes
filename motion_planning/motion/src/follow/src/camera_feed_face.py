#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def republish(msg):
    """Node that publishes camera feed to baxter's face screen topic
    """

    display_publisher.publish(msg)

rospy.init_node("camera_feed_face")
display_publisher = rospy.Publisher("/robot/xdisplay", Image, queue_size=10)
sub = rospy.Subscriber("/baxter_image", Image, republish, None, 1)
rospy.spin()
