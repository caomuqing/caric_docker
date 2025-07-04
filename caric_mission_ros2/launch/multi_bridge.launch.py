#!/usr/bin/env python3
"""
Multi-Domain Bridge Launch File

Main entry point for launching the complete CARIC mission system.
This includes PPCom services and domain bridges.

Usage:
ros2 launch caric_mission_ros2 multi_bridge.launch.py

This will start:
- PPCom services (router and call services)
- Domain bridges for all configured locations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for the complete mission system."""
    
    # Package name argument
    package_name_arg = DeclareLaunchArgument(
        'package_name',
        default_value='caric_mission_ros2',
        description='Name of the package containing the config files'
    )
    
    # PPCom enable argument
    enable_ppcom_arg = DeclareLaunchArgument(
        'enable_ppcom',
        default_value='true',
        description='Enable PPCom services'
    )
    
    # Bridges enable argument
    enable_bridges_arg = DeclareLaunchArgument(
        'enable_bridges',
        default_value='true',
        description='Enable domain bridges'
    )
    
    # Get launch configuration values
    package_name = LaunchConfiguration('package_name')
    
    # Include PPCom Sequential Launcher
    ppcom_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'ppcom_launcher.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_ppcom'))
    )
    
    # Include Domain Bridges
    bridges_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'bridges.launch.py'
            ])
        ]),
        launch_arguments={
            'package_name': package_name,
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_bridges'))
    )

    return LaunchDescription([
        # Launch arguments
        package_name_arg,
        enable_ppcom_arg,
        enable_bridges_arg,

        # PPCom services (launched first)
        ppcom_launcher,
        
        # Domain bridges
        bridges_launcher,
    ])