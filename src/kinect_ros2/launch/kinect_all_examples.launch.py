#!/usr/bin/env python3
"""
kinect_all_examples.launch.py
==============================
Launches the camera driver plus all six example nodes.
Useful for live demos and classroom sessions.

Usage:
    ros2 launch kinect_ros2 kinect_all_examples.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('kinect_ros2')
    drv_p = os.path.join(pkg, 'config', 'kinect_params.yaml')
    ex_p = os.path.join(pkg, 'config', 'examples_params.yaml')

    # Launch arguments
    rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='30.0',
        description='Camera frame rate [Hz]')
    tilt_arg = DeclareLaunchArgument(
        'tilt_angle', default_value='0.0',
        description='Kinect tilt [-31..31 degrees]')

    # Camera driver
    camera = Node(
        package='kinect_ros2',
        executable='kinect_driver',
        name='kinect_driver_node',
        parameters=[drv_p, {
            'publish_rate': LaunchConfiguration('publish_rate'),
            'tilt_angle': LaunchConfiguration('tilt_angle'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # Helper to create example node
    def ex_node(exe, name):
        return Node(
            package='kinect_ros2',
            executable=exe,
            name=name,
            parameters=[ex_p],
            output='screen',
            emulate_tty=True,
        )

    # Stagger example starts to avoid flooding at t=0
    rgb_ex   = TimerAction(period=1.0,  actions=[ex_node('ex01_rgb_viewer',      'rgb_viewer_node')])
    ruler_ex = TimerAction(period=1.5,  actions=[ex_node('ex02_depth_ruler',     'depth_ruler_node')])
    hist_ex  = TimerAction(period=2.0,  actions=[ex_node('ex03_depth_histogram', 'depth_histogram_node')])
    hand_ex  = TimerAction(period=2.5,  actions=[ex_node('ex04_hand_detector',   'hand_detector_node')])
    cloud_ex = TimerAction(period=3.0,  actions=[ex_node('ex05_coloured_cloud',  'coloured_cloud_node')])
    avoid_ex = TimerAction(period=3.5,  actions=[ex_node('ex06_obstacle_avoidance', 'obstacle_avoidance_node')])

    return LaunchDescription([
        rate_arg,
        tilt_arg,
        LogInfo(msg='=== kinect_ros2 v2 — All Examples Launch ==='),
        camera,
        rgb_ex,
        ruler_ex,
        hist_ex,
        hand_ex,
        cloud_ex,
        avoid_ex,
        LogInfo(msg='All nodes scheduled. Watch for [INFO] confirmations above.'),
    ])
