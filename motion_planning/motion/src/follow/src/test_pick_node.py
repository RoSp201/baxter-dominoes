#!/usr/bin/env python
import sys
import tf
import rospy
from tf2_msgs.msg import TFMessage
import ar_tag_pos as arp
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface import gripper as baxter_gripper
import moveit_msgs.msg
from follow.srv import Translate, PickNPlace

global target_tag

def follow(msg):
    
    marker = None
    for m in msg.markers:
        if m.id == target_tag:
            marker = m

    if not marker:
        print("Ar tag {} not found.".format(target_tag))
        return
        
    moved = False
    while not moved:
        print "\nCamera pose of AR tag: \n", marker.pose.pose

        #TEST: Test new translate coordinates service for ar tag to base frame, returns transformed PoseStamped()
        rospy.wait_for_service("translate_server")
        resp = PoseStamped()
        try:
            translate = rospy.ServiceProxy("translate_server", Translate)
            resp = translate(marker.pose, 'left_hand_camera_axis')
            print "\nTransformed base pose of AR tag: \n", resp.output_pose_stamped.pose
            rospy.sleep(1.0)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        print "\n\nPASSED: Coordinate Transform service\n\n"


        moved = True

        #just general pose to place tag at for testing purposes
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base"
        goal_pose.pose.position.x = 0.5
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = -1.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        start_pose = PoseStamped()
        start_pose = resp.output_pose_stamped

        #TEST: See if pick and place service works correctly
        rospy.wait_for_service("pick_n_place_server")
        try:
            left = sys.argv[2]
            pick_n_place = rospy.ServiceProxy("pick_n_place_server", PickNPlace)
            response = pick_n_place(start_pose, goal_pose, left)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.sleep(1.0)
        print("\n\nPASSED: pick_n_place_service call.")
        print("\nexiting...")
    rospy.signal_shutdown("Moved arm")
    

def test_pick_node():

    rospy.init_node("test_pick_node", anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit('Use: test_pick_node.py [AR tag number] [orientation (L or R)]')
    else:
        target_tag = int(sys.argv[1])
        test_pick_node()



