#!/bin/bash
workspace=$(pwd)

shell_type=${SHELL##*/}
shell_config="source ./install/setup.$shell_type"
shell_exec="exec $shell_type"

gnome-terminal --title="realsense" -- $shell_type -c "$shell_config; ros2 run realsense2_camera list_camera_node; $shell_exec"