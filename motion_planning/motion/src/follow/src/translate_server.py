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
    String frame
    ---
    PoseStamped output_pose_stamped

    """
    #this function will turn camera coordinates into base frame coordinates 
    #we can actually make the gripper orientation in terms of the ar tag frame orientation as seen by the base frame
    
    #input will be a poseStamped() object referenced in the camera frame

    #make sure two lines below are uncommented in package.xml
    #also add the same into CMakeList.txt
    #<build_depend>message_generation</build_depend>
    #<run_depend>message_runtime</run_depend>

    x = coords.pose_stamped.pose.position.x
    y = coords.pose_stamped.pose.position.y
    z = coords.pose_stamped.pose.position.z

    frame = coords.frame
    print "Frame to transform: ", frame

    input_coords = np.array([x,y,z,1])

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    found = False

    while not found:
        base_gripper_pose_stamped = PoseStamped()
    	try:
            #transform frame coords, so with respect to base frame 
            #note, can use ar tag frame, so that gripper will always be oriented correctly with the ar tag it is picking up
            #tf_listener.waitForTransform("base", frame, rospy.Time(0), rospy.Duration(4.0))
            #transform = tf_listener.lookupTransform("base", frame, rospy.Time(0))
            #(trans, rot) = transform
            #tf_listener.waitForTransform("base", frame, rospy.Time(0), rospy.Duration(4.0))
            transform = tf_buffer.lookup_transform("base",
                                       "left_hand_camera_axis", #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            base_gripper_pose_stamped = tf2_geometry_msgs.do_transform_pose(coords.pose_stamped, transform)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print "Cam coordinates: ", x, y, z
        found = True
        #rbt = arp.return_rbt(trans=trans, rot=rot)
        #transforms the entire pose, not just position
        

        #original method if above function doesn't work properly
        #base_coords = rbt.dot(coords_cam_frame)
        #base_coords = base_coords.reshape(4,1)
        #x2 = base_coords.item(0)
        #y2 = base_coords.item(1)
        #z2 = base_coords.item(2)
        #print "Base coordinates: ", x2, y2, z2
        
        output_pose_stamped = PoseStamped()
        #output_pose_stamped.header.frame_id = "base"
        #output_pose_stamped.pose.position.x = x2
        #output_pose_stamped.pose.position.y = y2
        #output_pose_stamped.pose.position.z = z2

        #if want new method, just uncomment line below and switch out:

        return base_gripper_pose_stamped

        #return return output_pose 


def translate_server():
    rospy.init_node("pose_translate_server")
    s = rospy.Service("pose_translate_server", Translate, handle_translate)
    print "\n\nCoordinate translator server ready to use!\n\n" 
    rospy.spin()

if __name__ == "__main__":
    translate_server()

