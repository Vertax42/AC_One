#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

# AC one
terminator --new-tab --title="AC_One" --working-directory="${workspace}/../../../ROS2/X5_ws" -e "bash -c 'source /opt/ros/humble/setup.bash; rm -rf build install log .catkin_workspace src/CMakeLists.txt && colcon build; exec bash'"
sleep 0.5

# joy
terminator --new-tab --title="Joy" --working-directory="${workspace}/../../../arx_joy" -e "bash -c 'source /opt/ros/humble/setup.bash; rm -rf build install log .catkin_workspace src/CMakeLists.txt && colcon build; exec bash'"
sleep 0.5

# Realsense
terminator --new-tab --title="Joy" --working-directory="${workspace}/../../../realsense" -e "bash -c 'source /opt/ros/humble/setup.bash; rm -rf build install log .catkin_workspace src/CMakeLists.txt && colcon build; exec bash'"
sleep 0.5
# VR
# terminator --new-tab --title="VR" --working-directory="${workspace}/../../../ARX_VR_SDK/ROS2" -e "bash -c 'source /opt/ros/humble/setup.bash; ./port.sh; exec bash'"

