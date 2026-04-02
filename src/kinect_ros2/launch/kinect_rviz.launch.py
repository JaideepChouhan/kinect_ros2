#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('kinect_ros2')
    
    # Include the full stack launch
    full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'kinect_full.launch.py')
        )
    )
    
    # RViz2 node
    rviz_config = os.path.join(pkg, 'config', 'kinect.rviz')
    
    # Check if RViz config exists, if not use default
    if not os.path.exists(rviz_config):
        rviz_config = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if rviz_config else [],
        output='screen',
    )
    
    return LaunchDescription([
        full_launch,
        rviz_node,
    ])
