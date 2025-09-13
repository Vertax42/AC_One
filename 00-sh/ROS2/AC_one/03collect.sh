#!/bin/bash

workspace=$(pwd)

# workspace=/home/arx-4070ti/gitee/X5_SDK_KDL/00-sh/ROS2/2025

source ~/.bashrc

# CAN
gnome-terminal -t "can" -x sudo bash -c "cd ${workspace};cd ../../.. ; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
gnome-terminal -t "can" -x sudo bash -c "cd ${workspace};cd ../../.. ; cd ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
sleep 1
#x7s

# /home/arx/gitee/X5_SDK_KDL/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/launch/x5_v2/v2_remote_master.launch.py

gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../../..; cd ROS2/X5_ws; source install/setup.bash && ros2 launch arx_x5_controller v2_collect.launch.py; exec bash;"
sleep 0.1
#VR
# gnome-terminal -t "unity_tcp" -x  bash -c "cd ${workspace}; cd ../../..; cd ARX_VR_SDK/ROS2; source install/setup.bash;ros2 run serial_port serial_port_node;exec bash;"
# sleep 0.1
# gnome-terminal -t "arx5_pos_cmd" -x  bash -c "cd ${workspace}; cd ../../..; cd ARX_VR_SDK/ROS2; source install/setup.bash;ros2 topic echo /ARX_VR_L;exec bash;"
