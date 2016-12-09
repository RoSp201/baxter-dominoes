#!/usr/bin/env python
import sys
from math import pi
import tf
import rospy
from tf2_msgs.msg import TFMessage
import ar_tag_pos as arp
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose#, OrientationConstraint, Constraints
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface import gripper as baxter_gripper
import moveit_msgs.msg 

global target_tag

#this node will subscribe to /ar_pose_marker once and then command Baxter 
#try and move his left gripper to just above that location (the position of the AR tag in base frame)

def pick_and_place(goal_pose, flip=False):
    waypoints = []
    waypoints.append(goal_pose)
    (plan, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    print "fraction: ", fraction
    left_arm.execute(plan)
    rospy.sleep(3.0)



def follow(msg):
    
    #Wait for the IK service to become available
    #rospy.wait_for_service('compute_ik') 
    #Create the function used to call the service
    #compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
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
    #request = GetPositionIKRequest()
    #request.ik_request.group_name = "left_arm"
    #request.ik_request.ik_link_name = "left_gripper"
    #request.ik_request.attempts = 30
    #request.ik_request.pose_stamped.header.frame_id = "base" 

    tf_listener = tf.TransformListener()
    moved = False

    while not moved:

        print "Cam coordinates: ", x1, y1, z1
        try:
            #transform frame coords, so with respect to base frame
            tf_listener.waitForTransform("base", "left_hand_camera_axis", rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = tf_listener.lookupTransform("base", "left_hand_camera_axis", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        moved = True
        rbt = arp.return_rbt(trans=trans, rot=rot)
        base_coords = rbt.dot(coords_cam_frame)
        base_coords = base_coords.reshape(4,1)

        x2 = base_coords.item(0)
        y2 = base_coords.item(1)
        z2 = base_coords.item(2)
        print "Base coordinates: ", x2, y2, z2


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

        #Set the desired orientation for the end effector HERE
        #request.ik_request.pose_stamped.pose.position.x = x2
        #request.ik_request.pose_stamped.pose.position.y = y2
        #request.ik_request.pose_stamped.pose.position.z = z2 + 0.20
        #request.ik_request.pose_stamped.pose.orientation.x = 0.0
        #request.ik_request.pose_stamped.pose.orientation.y = -1.0
        #request.ik_request.pose_stamped.pose.orientation.z = 0.0
        #request.ik_request.pose_stamped.pose.orientation.w = 0.0  

        #response = compute_ik(request)
        #group = MoveGroupCommander("left_arm")

        

        #WAYPOINTS
        goal = Pose()
        goal.position.x = x2
        goal.position.y = y2
        goal.position.z = z2 + 0.10
        goal.orientation.x = 0.0
        goal.orientation.y = -1.0
        goal.orientation.z = 0.0
        goal.orientation.w = 0.0 

        goal2 = goal
        goal2.position.z = z2 + 0.005
        goal4 = goal2

        goal3 = Pose()
        goal3.position.x = x2
        goal3.position.y = y2
        goal3.position.z = z2 + 0.20
        goal3.orientation.x = 0.0
        goal3.orientation.y = -1.0
        goal3.orientation.z = 0.0
        goal3.orientation.w = 0.0 

        goal7 = Pose()
        goal7.position.x = x2
        goal7.position.y = y2
        goal7.position.z = z2 + 0.10
        goal7.orientation.x = 0.0
        goal7.orientation.y = -1.0
        goal7.orientation.z = 0.0
        goal7.orientation.w = 0.0  

        roscpp_initialize(sys.argv)
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        left_arm = MoveGroupCommander('left_arm')
        left_arm.set_planner_id('RRTConnectkConfigDefault')
        left_arm.set_planning_time(10)
        left_gripper = baxter_gripper.Gripper('left')
        left_arm.allow_replanning(True)
        left_arm.set_end_effector_link("left_gripper")
        left_arm.set_pose_reference_frame('base')
        left_gripper.set_vacuum_threshold(2.0)

        print("current pose of eof: {}".format(left_arm.get_current_pose("left_gripper")))

        #Create a path constraint for the arm
        # UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
        #orien_const = moveit_msgs.msg.OrientationConstraint()
        #orien_const.link_name = "left_gripper";
        
        #orien_const.header.frame_id = "base";
        #orien_const.orientation.y = -1.0;
        #orien_const.absolute_x_axis_tolerance = 0.02;
        #orien_const.absolute_y_axis_tolerance = 0.02;
        #orien_const.absolute_z_axis_tolerance = 0.02;
        #orien_const.weight = 1.0;
        #consts = moveit_msgs.msg.Constraints()
        #consts.orientation_constraints = [orien_const]
        #left_arm.set_path_constraints(consts)

        ### Move end effector to 10 cm above target (fast motion with less waypoints) ###
        waypoints = []
        waypoints.append(goal)
        (plan1, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print "fraction: ", fraction
        left_arm.execute(plan1)
        rospy.sleep(3.0)

        # goal1 = Pose()
        # goal1.position.x = x2
        # goal1.position.y = y2
        # goal1.position.z = z2 + 0.10
        # goal1.orientation.x = 
        # goal1.orientation.y = 0.0
        # goal1.orientation.z = 0.0
        # goal1.orientation.w = 0.0
        # waypoints = []
        # waypoints.append(goal1)
        # (plan11, fraction) = left_arm.compute_cartesian_path(
        #                            waypoints,   # waypoints to follow with end 
        #                            0.01,        # eef_step
        #                            0.0)         # jump_threshold
        # print "fraction: ", fraction
        # left_arm.execute(plan11)
        # rospy.sleep(3.0)


        ######## Pick Up Object Once safely above ##############
        #Precisely pick up object by increasing number of waypoints
        
        goal2 = Pose()
        goal2.position.x = x2
        goal2.position.y = y2
        goal2.position.z = z2 + 0.005
        goal2.orientation.x = 0.0
        goal2.orientation.y = -1.0
        goal2.orientation.z = 0.0
        goal2.orientation.w = 0.0 

        goal4 = goal2

        waypoints = []
        waypoints.append(goal2)
        (plan2, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print "fraction2: ", fraction
        left_arm.execute(plan2)
        rospy.sleep(1.0)
        
        waypoints = []
        waypoints.append(goal3)
        (plan3, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        ### Lift Object With suction gripper ###
        print "fraction3: ", fraction
        print("\npicking up tag")

        
        left_gripper.close(block=False)
        print("\nvaccuum sensor reading: {}".format(left_gripper.vacuum_sensor()))
        rospy.sleep(0.5)
        left_arm.execute(plan3)
        print("\nlifting tag")
        rospy.sleep(1.0)


        #rotate the domino 180 degrees of current position
        print("current pose of eof: {}".format(left_arm.get_current_pose("left_gripper")))

        goal5 = Pose()
        goal5.position.x = x2
        goal5.position.y = y2
        goal5.position.z = z2 + 0.15
        goal5.orientation.x = 0.0
        goal5.orientation.y = -1.0
        goal5.orientation.z = 0.0
        goal5.orientation.w = 0.0
                
        print("\nspinning domino in place")
        waypoints = []
        waypoints.append(goal5)
        (plan5, fraction5) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print "fraction5: ", fraction
        left_arm.execute(plan5)
        rospy.sleep(1.0)

        #set the domino back down without dropping to avoid misplacement
        print("\nsetting back down")
        waypoints = []
        waypoints.append(goal4)
        (plan4, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print "fraction4: ", fraction
        left_arm.execute(plan4)
        rospy.sleep(1.0)
        print('Opening...')
        left_gripper.open(block=True)
        rospy.sleep(0.5)

        #go back to staging position
        print("\ngoing back to staging position for next pick")
        waypoints = []
        waypoints.append(goal3)
        (plan6, fraction) = left_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow with end 
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print "fraction6: ", fraction
        left_arm.execute(plan6)
        rospy.sleep(1.0)


        print("done.")

    rospy.signal_shutdown("Moved arm")
    

def listener():

    rospy.init_node("ik_from_ar_pos", anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, follow)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('Use: ik_from_ar_tag_pos.py [AR tag number for goal]')
    else:
        target_tag = int(sys.argv[1])
        listener()



