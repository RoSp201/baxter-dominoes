# Put all bash commands you have to run to set up the workspace here.

# Set up the game engine service:
cd game_engine
catkin_create_pkg game_engine rospy std_msgs

catkin_make
