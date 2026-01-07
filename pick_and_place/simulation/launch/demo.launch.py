#!/usr/bin/env python3
"""
Launch file for D1-550 robot demo (automated movement without joint GUI)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('d1_550_description')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'd1_550_description.urdf')
    
    # Path to RViz config
    rviz_config = os.path.join(pkg_dir, 'urdf.rviz')
    
    # Read URDF content (skip first line with XML declaration)
    with open(urdf_file, 'r') as f:
        lines = f.readlines()
        robot_description_content = ''.join(lines[1:])  # Skip XML declaration
    
    return LaunchDescription([
        # Robot State Publisher - publishes TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),
        
        # RViz2 - visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        
        # Environment Publisher - visualizes stages and box
        Node(
            package='d1_550_description',
            executable='environment_publisher.py',
            name='environment_publisher',
            output='screen'
        ),
    ])
