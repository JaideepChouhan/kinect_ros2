#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect_full.launch.py  –  Full Kinect stack with built-in /scan.

ROOT CAUSE OF LIBUSB_ERROR_BUSY (fixed here)
─────────────────────────────────────────────
The original launch used PushRosNamespace('kinect') and launched
tilt_control_node in the same group.  tilt_control_node opens its own
freenect context on the same USB device as kinect_driver_node.  Two
freenect contexts on the same device → libusb serialisation failure →
LIBUSB_ERROR_BUSY → sync_get_video/depth returns None → the
"cannot unpack non-iterable NoneType" errors fill the terminal.

FIX:
  1. tilt_control is now launched WITHOUT a namespace and with a 2-second
     delay so kinect_driver has time to fully claim the USB device first.
     tilt_control uses a different freenect API path (motor sub-device)
     which on most kernels can coexist with the camera sub-device IF the
     camera is opened first.  The 2 s delay ensures this ordering.

  2. If you still see LIBUSB_ERROR_BUSY with tilt, set enable_tilt:=false.
     The /scan and image topics will work perfectly without it.

  3. The old `depthimage_to_laserscan` package node is gone.
     /scan is now published directly by kinect_driver_node.
     Just set publish_scan: true in kinect_params.yaml (default: true).

USAGE
──────
  # Full stack – all topics including /scan:
  ros2 launch kinect_ros2 kinect_full.launch.py

  # Without tilt (safest, avoids all USB contention):
  ros2 launch kinect_ros2 kinect_full.launch.py enable_tilt:=false

  # Without depth alert:
  ros2 launch kinect_ros2 kinect_full.launch.py enable_alert:=false

  # Disable /scan if you don't need it:
  ros2 launch kinect_ros2 kinect_full.launch.py publish_scan:=false

TOPICS PUBLISHED
─────────────────
  /camera/rgb/image_raw          sensor_msgs/Image
  /camera/rgb/camera_info        sensor_msgs/CameraInfo
  /camera/depth/image_raw        sensor_msgs/Image
  /camera/depth/image_meters     sensor_msgs/Image   (float32, metres)
  /camera/depth/camera_info      sensor_msgs/CameraInfo
  /scan                          sensor_msgs/LaserScan
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg    = get_package_share_directory('kinect_ros2')
    params = os.path.join(pkg, 'config', 'kinect_params.yaml')

    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'device_index', default_value='0',
            description='Kinect device index (0 = first sensor)'),

        DeclareLaunchArgument(
            'target_fps', default_value='30.0',
            description='Camera capture rate [Hz]'),

        DeclareLaunchArgument(
            'publish_scan', default_value='true',
            description='Publish /scan from depth image (built-in, no extra node)'),

        DeclareLaunchArgument(
            'enable_alert', default_value='true',
            description='Enable depth proximity alert node'),

        # FIX: tilt defaults to FALSE to avoid LIBUSB_ERROR_BUSY.
        # Enable only if your hardware works with both contexts open.
        DeclareLaunchArgument(
            'enable_tilt', default_value='false',
            description='Enable tilt control node (may cause LIBUSB_ERROR_BUSY '
                        'on some setups – leave false if you see that error)'),

        DeclareLaunchArgument(
            'log_level', default_value='info'),

        LogInfo(msg='[kinect_full] Launching Kinect ROS2 full stack …'),

        # ── 1. Core driver (owns the USB device exclusively) ──────────────
        # No namespace – topics publish at /camera/... and /scan directly.
        # Namespace was the second reason for confusion in the original code
        # (topics ended up at /kinect/camera/... making remappings messy).
        Node(
            package='kinect_ros2',
            executable='kinect_driver',
            name='kinect_driver_node',
            output='screen',
            parameters=[
                params,
                {
                    # Override from launch args (passed as strings – correct)
                    'device_index': LaunchConfiguration('device_index'),
                    'target_fps':   LaunchConfiguration('target_fps'),
                    # publish_scan comes from kinect_params.yaml by default;
                    # override here so the launch arg takes effect
                    'publish_scan': LaunchConfiguration('publish_scan'),
                },
            ],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),

        # ── 2. Depth alert (optional, no USB access, safe to run in parallel) ─
        Node(
            package='kinect_ros2',
            executable='depth_alert',
            name='depth_alert_node',
            output='screen',
            parameters=[params],
            condition=IfCondition(LaunchConfiguration('enable_alert')),
        ),

        # ── 3. Tilt control (optional, delayed so driver owns USB first) ──
        # The 2-second delay gives kinect_driver time to fully open the
        # camera sub-device before tilt_control opens the motor sub-device.
        # On most kernels these are separate libusb interfaces and coexist.
        # If you still see LIBUSB_ERROR_BUSY, set enable_tilt:=false.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='kinect_ros2',
                    executable='tilt_control',
                    name='tilt_control_node',
                    output='screen',
                    parameters=[params],
                    condition=IfCondition(LaunchConfiguration('enable_tilt')),
                ),
            ],
        ),

        LogInfo(msg='[kinect_full] All nodes launched.'),
        LogInfo(msg='[kinect_full] Topics: /camera/rgb/image_raw  '
                    '/camera/depth/image_meters  /scan'),
        LogInfo(msg='[kinect_full] Verify: ros2 topic hz /scan'),
    ])