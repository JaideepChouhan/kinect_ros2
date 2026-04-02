#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg = get_package_share_directory('kinect_ros2')
    params = os.path.join(pkg, 'config', 'kinect_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='kinect',
            description='Namespace for all Kinect topics'
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Kinect device index (0 = first sensor)'
        ),
        DeclareLaunchArgument(
            'enable_tilt',
            default_value='true',
            description='Enable tilt control node'
        ),
        DeclareLaunchArgument(
            'enable_laserscan',
            default_value='false',
            description='Enable depth-to-laserscan conversion'
        ),
        DeclareLaunchArgument(
            'enable_alert',
            default_value='true',
            description='Enable depth alert node'
        ),

        LogInfo(msg='Launching Kinect ROS2 full stack...'),

        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),

            # Core driver
            Node(
                package='kinect_ros2',
                executable='kinect_driver',
                name='kinect_driver_node',
                output='screen',
                parameters=[params, {
                    'device_index': LaunchConfiguration('device_index'),
                }],
            ),

            # Depth alert (optional)
            Node(
                package='kinect_ros2',
                executable='depth_alert',
                name='depth_alert_node',
                output='screen',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('enable_alert')),
            ),

            # Tilt control (optional)
            Node(
                package='kinect_ros2',
                executable='tilt_control',
                name='tilt_control_node',
                output='screen',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('enable_tilt')),
            ),

            # Depth-to-laserscan (optional)
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depth_to_laserscan',
                output='screen',
                parameters=[{
                    'output_frame': 'camera_depth_frame',
                    'scan_time': 0.033,
                    'range_min': 0.45,
                    'range_max': 4.0,
                    'scan_height': 60,
                }],
                remappings=[
                    ('image', '/camera/depth/image_raw'),
                    ('camera_info', '/camera/depth/camera_info'),
                ],
                condition=IfCondition(LaunchConfiguration('enable_laserscan')),
            ),
        ]),
    ])
