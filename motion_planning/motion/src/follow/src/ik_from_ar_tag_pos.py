#!/usr/bin/env python
import sys
import tf
import rospy
from tf2_msgs.msg import TFMessage
import ar_tag_pos as arp
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers

global subscriber
global target_tag

#this node will subscribe to /ar_pose_marker once and then command Baxter 
#try and move his left gripper to just above that location (the position of the AR tag in base frame)

def follow(msg):
    
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik') 
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    marker = None
    for m in msg.markers:
        if m.id == target_tag:
            marker = m

    if not marker:
        print("Ar tag {} not found.".format(target_tag))
        return

    x1 = marker.pose.pose.position.x
    y1 = marker.pose.pose.position.y
    z1 = marker.pose.pose.position.z

    #collected coordinates are observered with respect to camera frame.
    coords_cam_frame = np.array([x1,y1,z1,1])

    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base" 

    #orien_const = OrientationConstraint()
    #orien_const.link_name = "left_gripper";
    #orien_const.header.frame_id = "base";
    #orien_const.orientation.y = -1.0;
    #orien_const.absolute_x_axis_tolerance = 0.1;
    #orien_const.absolute_y_axis_tolerance = 0.1;
    #orien_const.absolute_z_axis_tolerance = 0.1;
    #orien_const.weight = 1.0;
    #consts = Constraints()
    #consts.orientation_constraints = [orien_const]

    flag = False
    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown(): 

        print "Cam coordinates: ", x1, y1, z1
        try:
            #transform frame coords, so with respect to base frame
            tf_listener.waitForTransform("base", "left_hand_camera", rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = tf_listener.lookupTransform("base", "left_hand_camera", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rbt = arp.return_rbt(trans=trans, rot=rot)
        base_coords = rbt.dot(coords_cam_frame)
        base_coords = base_coords.reshape(4,1)

        x2 = base_coords.item(0)
        y2 = base_coords.item(1)
        z2 = base_coords.item(2)
        print "Base coordinates: ", x2, y2, z2

        raw_input("Enter to execute move: ")

        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x2
        request.ik_request.pose_stamped.pose.position.y = y2
        request.ik_request.pose_stamped.pose.position.z = z2 + 0.2
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = -1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0  

        response = compute_ik(request)
        group = MoveGroupCommander("left_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Setting just the position without specifying the orientation
        #group.set_position_target([0.5, 0.0, 0.0])

        # Plan IK and execute
        if flag == False:
            print(response)
            group.go()
            print "tried to execute move."
            #flag = True
        else:
            print "\n\nAlready moved. Please Restart.\n\n"

        if response.error_code.val == 1:
            #flag = True
            pass
    subscriber.unregister()
    

def listener():

    rospy.init_node("ik_from_ar_pos", anonymous=True)
    subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    if len(sys.argv) < 2:
        sys.exit('Use: ik_from_ar_tag_pos.py [AR tag number for goal]')
    else:
        #ar_tags = {}
        #ar_tags['ar3'] = 'ar_marker_' + sys.argv[1]
        target_tag = int(sys.argv[1])
        listener()
