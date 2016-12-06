#!/bin/bash
echo "LAUNCHING ENABLE_BAX"
gnome-terminal -x sh -c "bash_scripts/baxter_untuck_camera_enable.sh; bash" &

sleep 5

echo "LAUNCHING RVIZ"
gnome-terminal -x sh -c "bash_scripts/rviz_setup.sh; bash" &

sleep 5

echo "LAUNCHING ACTION_SERVER"
gnome-terminal -x sh -c "bash_scripts/action_server_setup.sh; bash"

sleep 5

echo "LAUNCHING MOVEIT"
gnome-terminal -x sh -c "bash_scripts/moveit_setup.sh; bash"


# if [[ $# -gt 0 ]]; then
# 	gnome-terminal -x sh -c "bash_scripts/run_baxter.sh $@; bash"
# fi
