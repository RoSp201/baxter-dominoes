#!/bin/bash
echo "Ready to enable Baxter. Press [Enter]: "
read strt
echo "LAUNCHING ENABLE_BAX"
gnome-terminal -x sh -c "bash_scripts/baxter_untuck_camera_enable.sh; bash" &

sleep 2
echo "Ready to load rviz... press [Enter] to start: "
read rvizbuddy
echo "LAUNCHING RVIZ"
gnome-terminal -x sh -c "bash_scripts/rviz_setup.sh; bash" &

sleep 2
echo "LAUNCHING ACTION_SERVER"
gnome-terminal -x sh -c "bash_scripts/action_server_setup.sh; bash"

sleep 2
echo "Ready to load moveit. Wait for action server to load, then hit [Enter]: "
read mvit
echo "LAUNCHING MOVEIT"
gnome-terminal -x sh -c "bash_scripts/moveit_setup.sh; bash"


# if [[ $# -gt 0 ]]; then
# 	gnome-terminal -x sh -c "bash_scripts/run_baxter.sh $@; bash"
# fi
