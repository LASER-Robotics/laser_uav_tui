import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'laser_uav_tui'
    executable_name = 'status_tui' 

    return LaunchDescription([
        Node(
            package=package_name,
            executable=executable_name,
            name='uav_status_tui_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
            }]
        )
    ])