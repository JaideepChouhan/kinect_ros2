#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('kinect_ros2')
    params = os.path.join(pkg, 'config', 'kinect_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Kinect device index (0 = first sensor)'
        ),
        DeclareLaunchArgument(
            'target_fps',
            default_value='30.0',
            description='Target capture rate in Hz (max 30)'
        ),
        
        Node(
            package='kinect_ros2',
            executable='kinect_driver',
            name='kinect_driver_node',
            output='screen',
            parameters=[params, {
                'device_index': LaunchConfiguration('device_index'),
                'target_fps': LaunchConfiguration('target_fps'),
            }],
        ),
    ])
