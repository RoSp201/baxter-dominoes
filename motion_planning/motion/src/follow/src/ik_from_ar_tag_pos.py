#!/usr/bin/env python
import sys
import tf
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf
from ar_tag_subs import compute_twist, return_rbt
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from ar_track_alvar_msgs.msg import AlvarMarkers


#this node will subscribe to the tf topic and read in the transform of the ar tag seen by the left hand camera and then will turn these transform into cartesian coordinates. Then baxter will take the coordinates and pipe them to the ik solver which will try to go to that location (above it).


def follow(msg):
    
    print "CALLBACK BETCHES"
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik') 
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    raw_input("Hit enter")    
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    try:

        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z + 0.5
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0      
        
        response = compute_ik(request)
        
        #Print the response HERE
        print(response)
        group = MoveGroupCommander("left_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK and execute
        group.go()

        print("DONE!!!")

    except e:
        print "Service call failed:%s"%e

        
    subscriber.unregister()

def listener(ar_tags):
    '''listener for ar_pose_marker to find cart coords for baxter arm to follow
    '''
    rospy.init_node("ik_from_ar_pos")
    subscriber = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, follow)
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('Use: ik_from_ar_tag_pos.py [AR tag number for goal] ')
        sys.exit()

    ar_tags = {}
    ar_tags['ar3'] = 'ar_marker_' + sys.argv[1]
    print('\n\n was able to get here' )
    #callback function
    global subscriber
    listener(ar_tags=ar_tags)
