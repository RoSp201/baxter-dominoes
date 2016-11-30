
bash baxter.sh
source baxter_ws/devel/setup.bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_tools_camera_control.py -o left_hand_camera -r 1280x800
