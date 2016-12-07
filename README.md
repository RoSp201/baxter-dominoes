# README #

### Guide to using Baxter ###


```
#!


TO turn off baxter's sonar sensors for recording video without feedback:
rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0



Run ~/.bashrc in every new bash window or after sshing into baxter

Login team14:dominoespizza2016
Check ~/.bashrc for "source /opt/ros/indigo/setup.bash" "source /scratch/shared/baxter_ws/devel/setup.bash"
Go into workspace
Run baxter.sh when you want to use baxter
Might have to source setup.bash when ssh'ed into baxter

Baxter setup:
Enable robot with baxter_tools enable_robot.py -e
Untuck with baxter_tools tuck_arms -u
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800

Our setup:
roslaunch follow run_all.launch
rosrun rviz rviz
If not in saved config:
	add camera frame
	set image topic /cameras/left_hand_camera/image
	add tf frame
	set fixed frame in global options to base
	Under TF, uncheck "All Enabled", check "base" "left_gripper" "left_hand_camera" "ar_marker_#"
	If ar tag is in camera view, should automatically show up in list

For IK solver:
	rosrun baxter_interface joint_trajectory_action_server.py

	In new window:
		roslaunch baxter_moveit_config move_group.launch
		Should have crazy long output ending in "All is well!..."

*NOTE*
if you want to do setup all in one command and don't care about the output of services you can launch everything with the following commands in a baxter shell:
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
roslaunch follow run_all.launch &
rosrun rviz rviz &
rosrun baxter_interface joint_trajectory_action_server.py


In new window:
baxter.sh
source devel/setup.bash in motion_planning
rosrun follow ik_from_ar_tag_pos.py #
	where # is the integer ar tag number


wiki.ros.org/tf/TFUsingPython
http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
	transformPoint

To get information about seen ar_tag
rostopic echo ar_pose_marker
```
