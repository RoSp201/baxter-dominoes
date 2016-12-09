#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from baxter_interface import gripper as baxter_gripper
import moveit_msgs.msg 

def handle_pick_n_place(msg):
    """
    this is a service to help move dominoes to correct place position and orientation with respect to the board state
    inputs: PickNPlace.srv
        PoseStamped hand_domino
        PoseStamped target_location
    outputs: None

    """

    #intialization information for compute cartesian path
    roscpp_initialize(sys.argv)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    left_arm = MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    left_gripper = baxter_gripper.Gripper('left')
    left_arm.allow_replanning(True)
    left_gripper.set_vacuum_threshold(2.0)
    left_arm.set_end_effector_link("left_gripper")
    left_arm.set_pose_reference_frame('base')

    
    x = msg.hand_domino.pose.position.x
    y = msg.hand_domino.pose.position.y
    z = msg.hand_domino.pose.position.z

    #going to staging area above domino to be picked up, no rotation yet
    goal = Pose()
    goal.position.x = x
    goal.position.y = y
    goal.position.z = z + 0.10
    goal.orientation.x = 0.0
    goal.orientation.y = -1.0
    goal.orientation.z = 0.0
    goal.orientation.w = 0.0

    #touch domino, no rotation yet
    goal2 = Pose()
    goal2.position.x = x
    goal2.position.y = y
    goal2.position.z = z + 0.005
    goal2.orientation.x = 0.0
    goal2.orientation.y = -1.0
    goal2.orientation.z = 0.0
    goal2.orientation.w = 0.0 

    #in between gripper will close

    #raise domino, no rotation yet
    goal3 = Pose()
    goal3.position.x = x
    goal3.position.y = y
    goal3.position.z = z + 0.15
    goal3.orientation.x = 0.0
    goal3.orientation.y = -1.0
    goal3.orientation.z = 0.0
    goal3.orientation.w = 0.0 

    #make this goal next to where the domino should be placed and correct orientation
    #string checking comes in
    turn = 0
    goal4 = Pose()
    goal4.position.x = msg.target_locagion.pose.position.x 
    goal4.position.y = msg.target_location.pose.position.y
    goal4.position.z = z + 0.15
    
    if left_right == "L":
        turn = 1.0    #rotate counter clockwise (positive radians)
    else:
        turn = -1.0 #clockwise
    
    goal4.orientation.x = turn
    goal4.orientation.y = -1.0
    goal4.orientation.z = 0.0
    goal4.orientation.w = 0.0 

    #lowers gripper to place domino in final location, no rotation
    goal5 = Pose()
    goal5.position.x = msg.target_location.pose.position.x
    goal5.position.y = msg.target_location.pose.position.y
    goal5.position.z = z + 0.005
    goal5.orientation.x = turn
    goal5.orientation.y = -1.0
    goal5.orientation.z = 0.0
    goal5.orientation.w = 0.0  

    #raise gripper after placement, no rotation
    goal6 = Pose()
    goal6.position.x = msg.target_location.pose.position.x
    goal6.position.y = msg.target_location.pose.position.y
    goal6.position.z = z + 0.15
    goal6.orientation.x = turn
    goal6.orientation.y = -1.0
    goal6.orientation.z = 0.0
    goal6.orientation.w = 0.0

    #move back to staging position in hand area, includes reset rotation
    goal7 = msg.hand_domino
    goal7.position.z = 0.25

    print("starting pose of eof: {}".format(left_arm.get_current_pose("left_gripper")))

    waypoints = []
    waypoints.append(goal1)
    (plan1, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    print "fraction: ", fraction
    left_arm.execute(plan1)
    rospy.sleep(3.0)

    waypoints = []
    waypoints.append(goal2)
    (plan2, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    print "fraction 2: ", fraction
    left_arm.execute(plan2)
    rospy.sleep(1.0)
    
    waypoints = []
    waypoints.append(goal3)
    (plan3, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    print "fraction 3: ", fraction
    print "Turning on Suction."
    left_gripper.close(block=False)
    rospy.sleep(0.5)
    left_arm.execute(plan3)
    rospy.sleep(2.0)
            
    waypoints = []
    waypoints.append(goal4)
    (plan4, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step (can make 0 for no waypoints?)
                               0.0)         # jump_threshold
    print "fraction 4: ", fraction
    left_arm.execute(plan4)
    rospy.sleep(1.0)


    waypoints = []
    waypoints.append(goal5)
    (plan5, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    print "fraction 5: ", fraction
    left_arm.execute(plan5)
    rospy.sleep(1.0)

    print('Turning Off Suction.')
    left_gripper.open(block=False)
    rospy.sleep(0.5)

    waypoints = []
    waypoints.append(goal6)
    (plan6, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    print "fraction 6: ", fraction
    left_arm.execute(plan6)
    rospy.sleep(1.0)


    waypoints = []
    waypoints.append(goal7)
    (plan7, fraction) = left_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow with end 
                               0.01,       # eef_step
                               0.0)         # jump_threshold
    
    print "fraction 7: ", fraction
    left_arm.execute(plan7)
    rospy.sleep(1.0)


    print("done.")
    

def pick_n_place_server():

    rospy.init_node("pick_n_place_server")
    s = rospy.Service("pick_n_place_server", PickNPlace, handle_pick_n_place)
    print "Ready to pick and place!"
    rospy.spin()

if __name__ == '__main__':
     pick_n_place_server()



