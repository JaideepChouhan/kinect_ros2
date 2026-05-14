#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
nav2_kinect.launch.py  –  Full autonomous navigation stack for Kinect robot.

System architecture
────────────────────
  kinect_driver_node
      ↓ /camera/depth/image_meters (32FC1)
  depth_to_laserscan_node
      ↓ /scan (LaserScan)
  slam_toolbox ──────────────→ /map  (building mode)
      ↓ map → odom TF
  Nav2 (controller, planner, costmaps, BT)
      ↓ /cmd_vel (Twist)
  Arduino (micro-ROS / rosserial) ← /cmd_vel
      ↑ /odom (from wheel encoders)

TF tree:
  map → odom → base_footprint → base_link → kinect_depth_optical_frame
                                           → kinect_rgb_optical_frame
                                           → kinect_imu_frame

Two-phase workflow:
  Phase 1 – Mapping:
    ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=mapping
    Drive the robot around manually to build the map.
    Save: ros2 run nav2_map_server map_saver_cli -f ~/my_map

  Phase 2 – Navigation:
    ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=navigation \
        map:=$HOME/my_map.yaml
    Set a goal in RViz2 or via /navigate_to_pose action.

Usage
──────
  # Mapping (SLAM, no existing map needed):
  ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=mapping

  # Navigation (load a saved map):
  ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=navigation \
      map:=$HOME/my_map.yaml

  # Voice commands (DETROIT bridge on /kinect/speech/command):
  ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=mapping \
      enable_voice:=true

  # Custom robot size:
  ros2 launch kinect_ros2 nav2_kinect.launch.py \
      robot_radius:=0.20 scan_height:=30

Launch arguments
─────────────────
  mode              str    mapping    mapping | navigation
  map               str    ''         path to .yaml map file (navigation mode)
  robot_radius      float  0.18       robot footprint radius (metres)
  scan_height       int    20         rows sampled in depth_to_laserscan
  range_max         float  3.50       max laser scan range (metres)
  enable_rviz       bool   true       launch RViz2 with Nav2 config
  enable_voice      bool   false      enable DETROIT voice command bridge
  use_sim_time      bool   false
  log_level         str    info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Package directories ────────────────────────────────────────────────
    kinect_dir  = get_package_share_directory('kinect_ros2')
    nav2_dir    = get_package_share_directory('nav2_bringup')
    slam_dir    = get_package_share_directory('slam_toolbox')

    nav2_params_file  = os.path.join(kinect_dir, 'config', 'nav2_params.yaml')
    slam_params_file  = os.path.join(kinect_dir, 'config', 'slam_params.yaml')
    rviz_config_file  = os.path.join(kinect_dir, 'config', 'nav2_kinect.rviz')

    # ── Launch arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('mode',          default_value='mapping'),
        DeclareLaunchArgument('map',           default_value=''),
        DeclareLaunchArgument('robot_radius',  default_value='0.18'),
        DeclareLaunchArgument('scan_height',   default_value='20'),
        DeclareLaunchArgument('range_max',     default_value='3.50'),
        DeclareLaunchArgument('enable_rviz',   default_value='true'),
        DeclareLaunchArgument('enable_voice',  default_value='false'),
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('log_level',     default_value='info'),
    ]

    # ── t=0: kinect_driver_node ────────────────────────────────────────────
    kinect_driver = Node(
        package='kinect_ros2',
        executable='kinect_driver',
        name='kinect_driver_node',
        output='screen',
        parameters=[{'target_fps': 15.0}],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )

    # ── t=1: depth_to_laserscan ────────────────────────────────────────────
    depth_to_scan = TimerAction(period=1.0, actions=[
        Node(
            package='kinect_ros2',
            executable='depth_to_laserscan',
            name='depth_to_laserscan_node',
            output='screen',
            parameters=[{
                'scan_height':   LaunchConfiguration('scan_height'),
                'range_max':     LaunchConfiguration('range_max'),
                'range_min':     0.40,
                'scan_frame_id': 'kinect_depth_optical_frame',
                'use_min':       True,
                'output_topic':  '/scan',
            }],
        ),
    ])

    # ── Static TF publishers ───────────────────────────────────────────────
    # EDIT these translation values to match where the Kinect is mounted
    # on your robot (x=forward, y=left, z=up, in metres from base_link origin)
    static_tfs = [
        # base_footprint → base_link (robot body origin, often same frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_footprint', 'base_link'],
        ),
        # base_link → kinect_depth_optical_frame
        # Kinect is mounted 30 cm high, 5 cm forward from robot centre,
        # tilted 0° (adjust x, z, and rotation to match your mounting)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_kinect_depth',
            arguments=[
                '0.05',  # x  (forward from base_link)
                '0.0',   # y
                '0.30',  # z  (height of Kinect)
                # quaternion: no rotation (Kinect facing forward, level)
                '-0.5', '0.5', '-0.5', '0.5',
                'base_link', 'kinect_depth_optical_frame',
            ],
        ),
        # kinect_depth_optical_frame → kinect_rgb_optical_frame
        # (small offset; usually published by the driver)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_depth_to_rgb',
            arguments=['0.025', '0', '0', '0', '0', '0',
                       'kinect_depth_optical_frame', 'kinect_rgb_optical_frame'],
        ),
    ]

    # ── slam_toolbox (mapping mode) ────────────────────────────────────────
    slam_mapping = TimerAction(period=2.0, actions=[
        LogInfo(msg='[nav2_kinect] Starting slam_toolbox in MAPPING mode …'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time':     LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(
                __import__('launch.substitutions', fromlist=['PythonExpression'])
                .PythonExpression(
                    ["'", LaunchConfiguration('mode'), "' == 'mapping'"]
                )
            ),
        ),
    ])

    # ── Nav2 bringup (external SLAM, so slam=False) ─────────────────────────
    nav2 = TimerAction(period=3.0, actions=[
        LogInfo(msg='[nav2_kinect] Starting Nav2 …'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file':  nav2_params_file,
                'map':          LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam':         'False',      # external SLAM already running
                'autostart':    'true',
            }.items(),
        ),
    ])

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz = TimerAction(period=4.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            output='screen',
        ),
    ])

    # ── DETROIT voice bridge (optional) ────────────────────────────────────
    voice = TimerAction(period=5.0, actions=[
        LogInfo(msg='[nav2_kinect] Starting DETROIT voice bridge …'),
        Node(
            package='kinect_ros2',
            executable='voice_bridge',   # your DETROIT bridge node
            name='voice_bridge_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_voice')),
        ),
    ])

    return LaunchDescription([
        *args,
        LogInfo(msg='[nav2_kinect] Kinect autonomous navigation starting …'),
        kinect_driver,
        depth_to_scan,
        *static_tfs,
        slam_mapping,
        nav2,
        rviz,
        voice,
        LogInfo(msg='[nav2_kinect] All nodes launched. Drive robot to build map.'),
        LogInfo(msg='[nav2_kinect] Save map: '
                    'ros2 run nav2_map_server map_saver_cli -f ~/my_map'),
    ])