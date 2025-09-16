#!/bin/bash

workspace=$(pwd)

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# CAN
gnome-terminal -t "can1" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.3
gnome-terminal -t "can3" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
sleep 0.3

# Ac_one
gnome-terminal --title="lift" -x $shell_type -i -c "cd ../ROS2/X5_ws; source install/setup.bash; ros2 launch arx_x5_controller v2_joint_control.launch.py; $shell_exec"
sleep 1

# Realsense
gnome-terminal --title="realsense" -x $shell_type -i -c "cd ${workspace}; cd ../realsense; ./realsense.sh; $shell_exec"
sleep 4

# Inference
gnome-terminal --title="inference" -x $shell_type -i -c "cd ${workspace}; cd ../act; conda activate act; export CUDA_VISIBLE_DEVICES=0; python inference.py; $shell_exec"   
