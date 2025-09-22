#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 获取当前工作空间路径
    workspace_path = os.path.expanduser("~/AC_One/ROS2/X5_ws")

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
            # 启动第一个终端 - PICO XR Node (VR控制器数据)
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title=🎮 PICO XR Node",
                    "--geometry=80x24+100+100",
                    "--",
                    "bash",
                    "-c",
                    f'cd "{workspace_path}" && '
                    "source install/setup.bash && "
                    'echo "================================================" && '
                    'echo "🎮 PICO XR Node - VR Controller Data Provider" && '
                    'echo "================================================" && '
                    'echo "This terminal shows VR controller connection status" && '
                    'echo "" && '
                    "ros2 run pico_xr_teleop pico_xr_node; "
                    'echo ""; '
                    'echo "🎮 PICO XR Node finished. Press Enter to close..."; '
                    "read",
                ],
                output="screen",
            ),
            # 延迟启动第二个终端 - PICO XR Teleop Node (遥操作节点)
            TimerAction(
                period=2.0,  # 延迟2秒启动
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "gnome-terminal",
                            "--title=🛡️ PICO XR Teleop & Safety Monitor",
                            "--geometry=80x30+600+100",
                            "--",
                            "bash",
                            "-c",
                            f'cd "{workspace_path}" && '
                            "source install/setup.bash && "
                            'echo "====================================================" && '
                            'echo "🛡️ PICO XR Teleop Node - Safety Monitor & Control" && '
                            'echo "====================================================" && '
                            'echo "This terminal shows safety status and teleop control info" && '
                            'echo "Watch for: 🟢 SAFE | 🔴 UNSAFE | ✅ Ready to grip!" && '
                            'echo "" && '
                            "ros2 run pico_xr_teleop pico_xr_teleop_node "
                            "--ros-args "
                            "-p position_scale:=1.0 "
                            "-p rotation_scale:=1.0; "
                            'echo ""; '
                            'echo "🛡️ PICO XR Teleop Node finished. Press Enter to close..."; '
                            "read",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
