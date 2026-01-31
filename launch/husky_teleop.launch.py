#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run Husky teleop (native Gazebo plugin version)
    Usage: ros2 launch husky_minimal husky_teleop.launch.py
    """
    
    # Teleop Twist Keyboard - publishes directly to /cmd_vel
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in new terminal window
        remappings=[
            ('cmd_vel', '/cmd_vel')  # Native plugin listens on /cmd_vel
        ]
    )
    
    return LaunchDescription([
        teleop_node,
    ])