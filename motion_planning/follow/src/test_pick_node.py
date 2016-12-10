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

        print "Cam pose of AR tag: ", marker.pose.pose

        #TEST: Test new translate coordinates service for ar tag to base frame, returns transformed PoseStamped()
        rospy.wait_for_service("pose_translate_server")
        resp = None
        try:
            tag_to_base_pose = rospy.ServiceProxy("pose_translate_server", Translate)
            resp = tag_to_base_pose(marker.pose, "ar_marker_"+str(target_tag))
            print "New Base pose of AR tag: ", resp.output_pose_stamped.pose
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e




        moved = True

        #intialization information for computer cartesian path
        roscpp_initialize(sys.argv)
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        left_arm = MoveGroupCommander('left_arm')
        left_arm.set_planner_id('RRTConnectkConfigDefault')
        left_arm.set_planning_time(10)
        left_gripper = baxter_gripper.Gripper('left')
        left_arm.allow_replanning(True)
        left_gripper.set_vacuum_threshold(2.0)
        left_arm.set_pose_reference_frame('base')
        left_gripper.calibrate()

        #just general pose to place tag at for testing purposes
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base"
        goal_pose.pose.position.x = 0.5
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.25
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = -1.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        start_pose = PoseStamped()
        start_pose = resp.output_pose_stamped
        start_pose.pose.position.z += 0.15 #to make sure clears table initially
        start_pose.pose.orientation.y = -1.0

        


        #TEST: See if pick and place service works correctly
        rospy.wait_for_service("pick_n_place_server")
        try:
            left = "L"
            pick_n_place = rospy.ServiceProxy("pick_n_place_server", PickNPlace)
            response = pick_n_place(start_pose, goal_pose, left)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e




        rospy.sleep(1.0)
        print("done.")
    rospy.signal_shutdown("Moved arm")
    

def listener():

    rospy.init_node("test_pick_node", anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('Use: test_pick_node.py [AR tag number for goal]')
    else:
        target_tag = int(sys.argv[1])
        listener()



