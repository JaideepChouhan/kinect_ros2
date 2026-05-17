#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect.launch.py  –  Minimal single-node launch.
Publishes all camera topics + /scan.  Nothing else.

Usage:
  ros2 launch kinect_ros2 kinect.launch.py
  ros2 launch kinect_ros2 kinect.launch.py target_fps:=15.0
  ros2 launch kinect_ros2 kinect.launch.py publish_scan:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg    = get_package_share_directory('kinect_ros2')
    params = os.path.join(pkg, 'config', 'kinect_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_index', default_value='0',
            description='Kinect device index'),
        DeclareLaunchArgument(
            'target_fps', default_value='30.0',
            description='Capture rate [Hz]'),
        DeclareLaunchArgument(
            'publish_scan', default_value='true',
            description='Publish /scan topic'),

        Node(
            package='kinect_ros2',
            executable='kinect_driver',
            name='kinect_driver_node',
            output='screen',
            parameters=[
                params,
                {
                    'device_index': LaunchConfiguration('device_index'),
                    'target_fps':   LaunchConfiguration('target_fps'),
                    'publish_scan': LaunchConfiguration('publish_scan'),
                },
            ],
        ),
    ])