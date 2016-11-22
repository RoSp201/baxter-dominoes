#!/usr/bin/env python
import sys
#import tf
import rospy
#from tf2_msgs.msg import TFMessage
#from geometry_msgs.msg import Transform, Vector3, Twist
#import exp_quat_func as eqf
#from ar_tag_subs import compute_twist, return_rbt
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from ar_track_alvar_msgs.msg import AlvarMarkers

#this node will subscribe to /ar_pose_marker once and then command Baxter 
#try and move his left gripper to just above that location (the position of the AR tag in base frame)

def follow(msg):
    
    print "callback message"
    
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

    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = z + 0.3
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0    
    
    try:
        response = compute_ik(request)
        print(response)
        group = MoveGroupCommander("left_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Setting just the position without specifying the orientation
        # group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK and execute
        group.go()

        print("DONE!")

    except e:
        print "Service call failed:%s"%e

    subscriber.unregister()



def listener(ar_tags):

    rospy.init_node("ik_from_ar_pos")
    subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)
    print("listener")
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    if len(sys.argv) < 2:
        sys.exit('Use: ik_from_ar_tag_pos.py [AR tag number for goal]')
    else:
        ar_tags = {}
        ar_tags['ar3'] = 'ar_marker_' + sys.argv[1]

        print('\nwas able to get here' )
        global subscriber
        listener(ar_tags)
