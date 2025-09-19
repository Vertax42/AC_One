import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pico_xr_teleop',
            executable='pico_xr_node',
            name='pico_xr_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])