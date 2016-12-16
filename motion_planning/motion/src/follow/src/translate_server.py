#!/usr/bin/env python
import sys
import rospy
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose
import ar_tag_pos as arp
import numpy as np
from follow.srv import Translate

def handle_translate(coords):
    """
    Translate.srv format:

    PoseStamped pose_stamped
    string frame
    ---
    PoseStamped output_pose_stamped

    """    
    x = coords.pose_stamped.pose.position.x
    y = coords.pose_stamped.pose.position.y
    z = coords.pose_stamped.pose.position.z

    frame = coords.frame
    print "\n\nFrame to transform: ", frame
    input_coords = np.array([x,y,z,1])

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_listener = tf.TransformListener()
    
    found = False

    while not found:
        base_gripper_pose_stamped = PoseStamped()
    	try:
            #tf_listener.waitForTransform("base", "left_hand_camera_axis", rospy.Time(0), rospy.Duration(7.0))
            #(trans, rot) = tf_listener.lookupTransform("base", "left_hand_camera_axis", rospy.Time(0))
            tf_listener.waitForTransform("base", frame, rospy.Time(0), rospy.Duration(4.0))
            transform = tf_buffer.lookup_transform("base",
                                       "left_hand_camera_axis", #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(7.0)) #wait for 7 seconds
            base_gripper_pose_stamped = tf2_geometry_msgs.do_transform_pose(coords.pose_stamped, transform)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print "Cam coordinates: ", x, y, z
        found = True
        print(base_gripper_pose_stamped)
        
        #original method if above function doesn't work properly
        #base_coords = rbt.dot(input_coords)
        #print(base_coords)
        #base_coords = base_coords.reshape(4,1)
        #x2 = base_coords.item(0)
        #y2 = base_coords.item(1)
        #z2 = base_coords.item(2)
        #print "Base coordinates: ", x2, y2, z2
        #
        #output_pose_stamped = PoseStamped()
        #output_pose_stamped.header.frame_id = "base"
        #output_pose_stamped.pose.position.x = x2
        #output_pose_stamped.pose.position.y = y2
        #output_pose_stamped.pose.position.z = z2

        ##give default orientation facing down with respect to baxter's base frame
        #output_pose_stamped.pose.orientation.x = rot[0]
        #output_pose_stamped.pose.orientation.y = rot[1]
        #output_pose_stamped.pose.orientation.z = rot[2]
        #output_pose_stamped.pose.orientation.w = rot[3]

        ##if want new method, just uncomment line below and switch out:
        ##return base_gripper_pose_stamped

        #return output_pose_stamped 
        return base_gripper_pose_stamped


def translate_server():
    rospy.init_node("translate_server")
    s = rospy.Service("translate_server", Translate, handle_translate)
    print "\n\nTranslator server ready!\n\n" 
    rospy.spin()

if __name__ == "__main__":
    translate_server()

