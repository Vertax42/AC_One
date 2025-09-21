#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "position_scale",
                default_value="1.0",
                description="Position scaling factor",
            ),
            DeclareLaunchArgument(
                "rotation_scale",
                default_value="1.0",
                description="Rotation scaling factor",
            ),
            # 启动pico_xr_node提供VR控制器数据
            Node(
                package="pico_xr_teleop",
                executable="pico_xr_node",
                name="pico_xr_node",
                output="screen",
            ),
            # 启动pico_xr_teleop_node进行遥操作
            Node(
                package="pico_xr_teleop",
                executable="pico_xr_teleop_node",
                name="pico_xr_teleop_node",
                output="screen",
                parameters=[
                    {
                        "position_scale": LaunchConfiguration("position_scale"),
                        "rotation_scale": LaunchConfiguration("rotation_scale"),
                    }
                ],
            ),
        ]
    )
