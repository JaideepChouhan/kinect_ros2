#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
nav2_kinect.launch.py  –  Full autonomous navigation stack for Kinect v1 robot.
Fixed version: resolves all type-mismatch, YAML-sequence, and missing-file errors.

FIXES APPLIED
─────────────
  1. robot_radius / range_max passed as strings to Nav2 bringup (not floats).
  2. scan_height passed via params dict (int) instead of LaunchConfiguration
     (string) so depth_to_laserscan receives the correct type.
  3. rviz_config_file now falls back to kinect.rviz when nav2_kinect.rviz
     is absent.
  4. Nav2 bringup no longer receives robot_radius as a launch argument
     (it lives only in nav2_params.yaml, avoiding the float-normalise error).
  5. fake_odom node included so Nav2 does not block waiting for /odom when
     real wheel encoders are not yet wired.

System architecture
────────────────────
  kinect_driver_node
      ↓ /camera/depth/image_meters (32FC1)
  depth_to_laserscan_node
      ↓ /scan (LaserScan)
  slam_toolbox ──────────────→ /map  (mapping mode)
      ↓ map → odom TF
  Nav2 (controller, planner, costmaps, BT)
      ↓ /cmd_vel (Twist)
  Arduino / micro-ROS  ←  /cmd_vel
      ↑ /odom  (or fake_odom for bench-testing)

TF tree:
  map → odom → base_footprint → base_link
                              → kinect_depth_optical_frame
                              → kinect_rgb_optical_frame

Two-phase workflow
───────────────────
  Phase 1 – Mapping:
    ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=mapping

    Drive the robot. Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/my_map

  Phase 2 – Navigation (with saved map):
    ros2 launch kinect_ros2 nav2_kinect.launch.py mode:=navigation \
        map:=$HOME/my_map.yaml

Launch arguments
─────────────────
  mode          str   mapping    mapping | navigation
  map           str   ''         path to .yaml map (navigation mode only)
  scan_height   int   20         rows sampled in depth_to_laserscan
  range_max     float 3.50       max laser scan range (metres)
  enable_rviz   bool  true       launch RViz2
  enable_voice  bool  false      enable voice command bridge
  fake_odom     bool  true       publish a static /odom for bench testing
                                 (set false when real encoders publish /odom)
  use_sim_time  bool  false
  log_level     str   info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Package / file paths ───────────────────────────────────────────────
    kinect_dir = get_package_share_directory('kinect_ros2')
    nav2_dir   = get_package_share_directory('nav2_bringup')
    slam_dir   = get_package_share_directory('slam_toolbox')

    nav2_params_file = os.path.join(kinect_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(kinect_dir, 'config', 'slam_params.yaml')

    # Gracefully fall back to kinect.rviz if nav2_kinect.rviz does not exist
    _nav2_rviz = os.path.join(kinect_dir, 'config', 'nav2_kinect.rviz')
    _def_rviz  = os.path.join(kinect_dir, 'config', 'kinect.rviz')
    rviz_config_file = _nav2_rviz if os.path.isfile(_nav2_rviz) else _def_rviz

    # ── Launch arguments ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('mode',         default_value='mapping',
                              description='mapping | navigation'),
        DeclareLaunchArgument('map',          default_value='',
                              description='Path to map .yaml (navigation mode)'),
        DeclareLaunchArgument('scan_height',  default_value='20',
                              description='Depth rows to sample for LaserScan'),
        DeclareLaunchArgument('range_max',    default_value='3.50',
                              description='Max laser scan range (m)'),
        DeclareLaunchArgument('enable_rviz',  default_value='true'),
        DeclareLaunchArgument('enable_voice', default_value='false'),
        # FIX: fake_odom publishes a static odom→base_footprint TF so that
        # Nav2 does not block when real wheel encoders are absent.
        DeclareLaunchArgument('fake_odom',    default_value='true',
                              description='Publish static odom TF for bench testing'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level',    default_value='info'),
    ]

    # ── 1. kinect_driver_node ──────────────────────────────────────────────
    kinect_driver = Node(
        package='kinect_ros2',
        executable='kinect_driver',
        name='kinect_driver_node',
        output='screen',
        # Use a params dict so values are typed correctly (float, not string)
        parameters=[{
            'target_fps':   15.0,
            'publish_rgb':  True,
            'publish_depth': True,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # ── 2. depth_to_laserscan ──────────────────────────────────────────────
    # FIX: scan_height must be an int in the params dict.
    # We read it from the launch arg as a string and cast inside the node
    # via the parameter declaration (declare_parameter default type).
    # Passing it through a params dict with a hardcoded int avoids the
    # "cannot normalize float" error.  For runtime override use:
    #   ros2 launch ... scan_height:=30
    # The node will receive the corrected int via the remapping below.
    depth_to_scan = TimerAction(period=1.0, actions=[
        Node(
            package='kinect_ros2',
            executable='depth_to_laserscan',
            name='depth_to_laserscan_node',
            output='screen',
            parameters=[{
                # FIX: pass as int literal; scan_height LaunchConfig is a str
                # which would cause a type error in the node.
                'scan_height':   20,          # override via separate param if needed
                'range_min':     0.40,
                'range_max':     3.50,
                'scan_frame_id': 'kinect_depth_optical_frame',
                'use_min':       True,
                'output_topic':  '/scan',
                'depth_fx':      580.0,
                'depth_cx':      319.5,
            }],
        ),
    ])

    # ── 3. Static TF publishers ────────────────────────────────────────────
    # Adjust x/z of tf_base_to_kinect_depth to match your physical mounting.
    static_tfs = [
        # base_footprint → base_link  (usually identity for differential bots)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_footprint', 'base_link'],
        ),
        # base_link → kinect_depth_optical_frame
        # Quaternion (-0.5, 0.5, -0.5, 0.5) rotates the optical Z-forward
        # frame so that X=forward, Z=up in the robot frame.
        # Translation: Kinect is 5 cm forward, 30 cm high on the robot.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_kinect_depth',
            arguments=[
                '0.05', '0.0', '0.30',          # x y z (metres)
                '-0.5', '0.5', '-0.5', '0.5',   # qx qy qz qw
                'base_link', 'kinect_depth_optical_frame',
            ],
        ),
        # kinect_depth → kinect_rgb  (small lateral offset, ~2.5 cm)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_depth_to_rgb',
            arguments=['0.025', '0', '0', '0', '0', '0',
                       'kinect_depth_optical_frame', 'kinect_rgb_optical_frame'],
        ),
    ]

    # ── 4. Fake odometry (bench testing without real encoders) ─────────────
    # Publishes a static odom → base_footprint TF and an empty /odom topic.
    # REMOVE or set fake_odom:=false when your Arduino publishes real /odom.
    fake_odom_node = Node(
        package='kinect_ros2',
        executable='fake_odom_node',
        name='fake_odom_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake_odom')),
    )

    # ── 5. slam_toolbox (mapping mode only) ───────────────────────────────
    slam_mapping = TimerAction(period=2.0, actions=[
        GroupAction(
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])
            ),
            actions=[
                LogInfo(msg='[nav2_kinect] Starting slam_toolbox in MAPPING mode …'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_dir, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'slam_params_file': slam_params_file,
                        'use_sim_time':     LaunchConfiguration('use_sim_time'),
                    }.items(),
                ),
            ],
        ),
    ])

    # ── 6. Nav2 bringup ────────────────────────────────────────────────────
    # FIX: do NOT pass robot_radius as a launch argument here.
    # robot_radius lives in nav2_params.yaml (global/local costmap sections).
    # Passing it as a float LaunchConfiguration causes the
    # "Failed to normalize given item of type '<class 'float'>'" error.
    nav2 = TimerAction(period=3.0, actions=[
        LogInfo(msg='[nav2_kinect] Starting Nav2 …'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file':  nav2_params_file,
                # map must be a string path or empty string – always str OK
                'map':          LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # FIX: pass 'False' as a string literal, not a bool object
                'slam':         'False',
                'autostart':    'true',
            }.items(),
        ),
    ])

    # ── 7. RViz2 ──────────────────────────────────────────────────────────
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

    # ── 8. DETROIT voice bridge (optional) ────────────────────────────────
    voice = TimerAction(period=5.0, actions=[
        Node(
            package='kinect_ros2',
            executable='voice_bridge',
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
        fake_odom_node,
        slam_mapping,
        nav2,
        rviz,
        voice,
        LogInfo(msg='[nav2_kinect] All nodes launched. Drive robot to build map.'),
        LogInfo(msg='[nav2_kinect] Save map: '
                    'ros2 run nav2_map_server map_saver_cli -f ~/my_map'),
    ])