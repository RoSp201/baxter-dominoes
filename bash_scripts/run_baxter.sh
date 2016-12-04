
bash baxter.sh
source ../motion_planning/motion/devel/setup.bash
if [[ $# -gt 0 ]]; then
	rosrun follow "$@"
else
	echo "Manually run baxter program."
fi