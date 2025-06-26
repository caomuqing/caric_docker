#!/usr/bin/env python3
"""
PPCom Sequential Launch File

This launch file starts ppcom_router_new.py first, waits for initialization,
then starts ppcom_call_new.py using ROS2 launch system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    """Generate launch description for PPCom sequential services"""
    
    # Use ROS2 Node action to launch the sequential launcher
    ppcom_sequential_launcher = Node(
        package='caric_mission_ros2',
        executable='ppcom_sequential_launcher.py',
        name='ppcom_sequential_launcher',
        output='screen',
    )
    
    return LaunchDescription([
        ppcom_sequential_launcher,
    ])