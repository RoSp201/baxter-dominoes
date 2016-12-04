#!/bin/bash
gnome-terminal -x sh -c "bash_scripts/baxter_untuck_camera_enable.sh; bash"

gnome-terminal -x sh -c "bash_scripts/rviz_setup.sh; bash"

gnome-terminal -x sh -c "bash_scripts/ik_setup.sh; bash"

# if [[ $# -gt 0 ]]; then
# 	gnome-terminal -x sh -c "bash_scripts/run_baxter.sh $@; bash"
# fi
