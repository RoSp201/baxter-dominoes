#!/usr/bin/env python
import sys
import tf
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf
import ar_tag_pos as arp
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from ar_track_alvar_msgs.msg import AlvarMarkers

global subscriber

#this node will subscribe to /ar_pose_marker once and then command Baxter 
#try and move his left gripper to just above that location (the position of the AR tag in base frame)

def follow(msg):

    li = tf.TransformListener()
    #transform frame coords, so with respect to base frame
    (trans, rot)  = li.lookupTransform("/cameras/left_hand_camera/left_hand_camera", "/cameras/left_hand_camera/base", rospy.Time(0))
    rbt = arp.return_rbt(trans=trans, rot=rot)
    
    global subscriber
    
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik') 
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    marker = msg.markers[0]
    x = marker.pose.pose.position.x
    y = marker.pose.pose.position.y
    z = marker.pose.pose.position.z
    print "Cam coordinates: ", x, y, z
    #collected coordinates are observered with respect to camera frame.
    coords_cam_frame = np.array([x,y,z,1])

    base_coords = rbt.dot(coords_cam_frame)
    print "Base coordinates: ",  base_coords

    #this will help pause 
    raw_input("Hit enter") 

    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 10
    request.ik_request.pose_stamped.header.frame_id = "base"

    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = 0.5#x + 0.3
    request.ik_request.pose_stamped.pose.position.y = 0.0
    request.ik_request.pose_stamped.pose.position.z = 0.5 #z + 0.5
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = -1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0    


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

    while not rospy.is_shutdown(): 
        try:
            response = compute_ik(request)
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.0, 0.0])

            # Plan IK and execute
            #group.go()
            if response.error_code.val != -31:
                flag = True
                print("DONE!")

        except rospy.ServiceException, e:
            print "Service call failed:%s"%e
    
        rospy.sleep(1)
    subscriber.unregister()



def listener(ar_tags):

    rospy.init_node("ik_from_ar_pos")
    subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)
    #print("listener")
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    if len(sys.argv) < 2:
        sys.exit('Use: ik_from_ar_tag_pos.py [AR tag number for goal]')
    else:
        ar_tags = {}
        ar_tags['ar3'] = 'ar_marker_' + sys.argv[1]

        #print('\nwas able to get here' )
        listener(ar_tags)






