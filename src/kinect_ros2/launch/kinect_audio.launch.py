#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect_audio.launch.py  –  Full Kinect v1 audio stack using libfreenect C API.

Architecture
────────────
  kinect_driver_node (Python, kinect_ros2)
      – Opens the Kinect camera/depth via FREENECT_DEVICE_CAMERA
      – Uploads the audio firmware to 045e:02ad as a side effect

  kinect_audio (C++, kinect_audio_cpp)
      – Opens the SAME Kinect via FREENECT_DEVICE_AUDIO only
      – No conflict: camera and audio are separate USB interfaces
      – Publishes /kinect/audio/raw (Int32MultiArray, 4-ch int32)
      –           /kinect/audio/cancelled (Int16MultiArray, int16 HW noise-cancelled)
      –           /kinect/audio/info (JSON)

  ex08_speech_recognition (Python, kinect_ros2)
      – Subscribes to /kinect/audio/cancelled → STT → /cmd_vel

  ex09_mic_visualizer (Python, kinect_ros2)
      – Subscribes to /kinect/audio/raw + /kinect/audio/cancelled → image

Startup order (staggered)
──────────────────────────
  t = 0.0 s   kinect_driver_node  (uploads audio firmware)
  t = 3.0 s   kinect_audio        (C++ node, audio firmware now loaded)
  t = 5.0 s   ex08_speech_recognition
  t = 5.5 s   ex09_mic_visualizer

Usage
─────
  # Full stack:
  ros2 launch kinect_ros2 kinect_audio.launch.py \\
      model_path:=$HOME/vosk-model-small-en-us-0.15

  # Audio driver only (no STT, no visualizer):
  ros2 launch kinect_ros2 kinect_audio.launch.py \\
      enable_speech:=false enable_viz:=false

  # kinect_driver already running in another terminal:
  ros2 launch kinect_ros2 kinect_audio.launch.py \\
      start_driver:=false

  # Google STT (needs internet):
  ros2 launch kinect_ros2 kinect_audio.launch.py backend:=google

  # View visualizer:
  ros2 run rqt_image_view rqt_image_view
  # select /kinect/audio/visualizer

Launch arguments
────────────────
  start_driver          bool   true
  device_index          int    0       Kinect device index
  backend               str    vosk    vosk | google | sphinx
  model_path            str    vosk-model-small-en-us-0.15
  vad_energy_thresh     int    300
  vad_silence_s         float  0.8
  linear_speed          float  0.25    m/s
  angular_speed         float  0.50    rad/s
  command_timeout_s     float  3.0
  render_rate_hz        float  5.0
  enable_speech         bool   true
  enable_viz            bool   true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('start_driver',       default_value='true'),
        DeclareLaunchArgument('device_index',        default_value='0'),
        DeclareLaunchArgument('backend',             default_value='vosk'),
        DeclareLaunchArgument('model_path',          default_value='vosk-model-small-en-us-0.15'),
        DeclareLaunchArgument('vad_energy_thresh',   default_value='300'),
        DeclareLaunchArgument('vad_silence_s',       default_value='0.8'),
        DeclareLaunchArgument('linear_speed',        default_value='0.25'),
        DeclareLaunchArgument('angular_speed',       default_value='0.50'),
        DeclareLaunchArgument('command_timeout_s',   default_value='3.0'),
        DeclareLaunchArgument('render_rate_hz',      default_value='5.0'),
        DeclareLaunchArgument('enable_speech',       default_value='true'),
        DeclareLaunchArgument('enable_viz',          default_value='true'),
    ]

    # ── t=0: kinect_driver_node ───────────────────────────────────────────
    # Opens FREENECT_DEVICE_CAMERA and uploads audio firmware as a side effect.
    driver = Node(
        package='kinect_ros2',
        executable='kinect_driver',
        name='kinect_driver_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    # ── t=3: kinect_audio (C++, kinect_audio_cpp package) ─────────────────
    # Uses FREENECT_DEVICE_AUDIO only – no conflict with driver or tilt node.
    audio = TimerAction(period=3.0, actions=[
        LogInfo(msg='[kinect_audio] Starting C++ audio node (libfreenect audio API) …'),
        Node(
            package='kinect_audio_cpp',
            executable='kinect_audio',
            name='kinect_audio_node',
            output='screen',
            parameters=[{
                'device_index': LaunchConfiguration('device_index'),
            }],
        ),
    ])

    # ── t=5: ex08_speech_recognition ─────────────────────────────────────
    speech = TimerAction(period=5.0, actions=[
        LogInfo(msg='[kinect_audio] Starting SpeechRecognitionNode …'),
        Node(
            package='kinect_ros2',
            executable='ex08_speech_recognition',
            name='speech_recognition_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_speech')),
            parameters=[{
                'backend':           LaunchConfiguration('backend'),
                'model_path':        LaunchConfiguration('model_path'),
                'vad_energy_thresh': LaunchConfiguration('vad_energy_thresh'),
                'vad_silence_s':     LaunchConfiguration('vad_silence_s'),
                'linear_speed':      LaunchConfiguration('linear_speed'),
                'angular_speed':     LaunchConfiguration('angular_speed'),
                'command_timeout_s': LaunchConfiguration('command_timeout_s'),
            }],
        ),
    ])

    # ── t=5.5: ex09_mic_visualizer ────────────────────────────────────────
    viz = TimerAction(period=5.5, actions=[
        LogInfo(msg='[kinect_audio] Starting MicVisualizerNode …'),
        Node(
            package='kinect_ros2',
            executable='ex09_mic_visualizer',
            name='mic_visualizer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_viz')),
            parameters=[{
                'render_rate_hz': LaunchConfiguration('render_rate_hz'),
            }],
        ),
    ])

    return LaunchDescription([
        *args,
        LogInfo(msg='[kinect_audio] t=0  kinect_driver_node (firmware upload) …'),
        driver,
        LogInfo(msg='[kinect_audio] t=3  kinect_audio C++ node …'),
        audio,
        speech,
        viz,
    ])

