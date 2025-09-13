#!/bin/bash

workspace=$(pwd)

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# CAN
gnome-terminal -t "can1" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.3
gnome-terminal -t "can3" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
sleep 0.3
gnome-terminal -t "can6" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can6.sh; exec bash;"
sleep 0.3

# Ac_one
gnome-terminal --title="lift" -x $shell_type -i -c "cd ../ROS2/X5_ws; source install/setup.bash; ros2 launch arx_x5_controller v2_collect.launch.py; $shell_exec"
sleep 0.3
gnome-terminal --title="joy" -x $shell_type -i -c "cd ../arx_joy; source install/setup.bash; ros2 run arx_joy arx_joy; $shell_exec"
sleep 0.2

# Realsense
gnome-terminal --title="realsense" -x $shell_type -i -c "cd ${workspace}; cd ../realsense; ./realsense.sh; $shell_exec"
sleep 3

# Collect
gnome-terminal --title="collect" -x $shell_type -i -c "cd ${workspace}; cd ../act; conda activate act; python collect.py --episode_idx -1 --num_episodes 20; $shell_exec"   
