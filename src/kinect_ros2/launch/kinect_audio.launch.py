#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect_audio.launch.py  –  Launches the full Kinect v1 audio stack.

Startup order (staggered to avoid CPU spikes):
  t=0.0s  kinect_audio_node      – mic capture, publishes raw + mono
  t=1.5s  ex08_speech_recognition_node – STT → /cmd_vel
  t=2.0s  ex09_mic_visualizer_node     – waveform/FFT image publisher

Usage examples
--------------
  # Minimal – just audio driver:
  ros2 launch kinect_ros2 kinect_audio.launch.py enable_speech:=false enable_viz:=false

  # Full stack with Vosk (offline STT):
  ros2 launch kinect_ros2 kinect_audio.launch.py \
      model_path:=/home/$USER/vosk-model-small-en-us-0.15

  # Google STT (needs internet):
  ros2 launch kinect_ros2 kinect_audio.launch.py backend:=google

  # View the visualizer in rqt:
  #   ros2 run rqt_image_view rqt_image_view
  #   (select /kinect/audio/visualizer)

  # Run alongside the depth/RGB driver:
  #   Terminal 1:  ros2 launch kinect_ros2 kinect.launch.py
  #   Terminal 2:  ros2 launch kinect_ros2 kinect_audio.launch.py

Launch arguments
----------------
  device_index       int    -1        PyAudio input device (-1 = auto)
  sample_rate        int    16000     Fixed by Kinect hardware
  chunk_frames       int    4096      PCM frames per ROS msg
  channels           int    4         4 = full array; 1 = mono
  backend            str    vosk      STT backend: vosk | google | sphinx
  model_path         str    vosk-model-small-en-us-0.15
  vad_energy_thresh  int    300       RMS energy threshold for VAD
  vad_silence_s      float  0.8       Silence duration triggering STT
  linear_speed       float  0.25      m/s
  angular_speed      float  0.50      rad/s
  command_timeout_s  float  3.0       Safety stop timeout
  render_rate_hz     float  5.0       Visualizer frame rate
  enable_speech      bool   true      Launch SpeechRecognitionNode
  enable_viz         bool   true      Launch MicVisualizerNode
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Declare all launch arguments ───────────────────────────────────────
    args = [
        # Audio driver
        DeclareLaunchArgument('device_index',  default_value='-1',
            description='PyAudio input device index (-1 = auto-detect Kinect mic)'),
        DeclareLaunchArgument('sample_rate',   default_value='16000',
            description='Mic sample rate in Hz (Kinect fixed at 16000)'),
        DeclareLaunchArgument('chunk_frames',  default_value='4096',
            description='PCM frames per ROS message (~256 ms at 16 kHz)'),
        DeclareLaunchArgument('channels',      default_value='4',
            description='Channels to capture (4 = full array; 1 = mono-only)'),
        # Speech recognition
        DeclareLaunchArgument('backend',       default_value='vosk',
            description='STT backend: vosk (offline) | google (online) | sphinx (offline)'),
        DeclareLaunchArgument('model_path',    default_value='vosk-model-small-en-us-0.15',
            description='Path to Vosk model directory (only used when backend=vosk)'),
        DeclareLaunchArgument('vad_energy_thresh', default_value='300',
            description='RMS energy above which audio counts as speech'),
        DeclareLaunchArgument('vad_silence_s', default_value='0.8',
            description='Seconds of silence after speech before STT runs'),
        DeclareLaunchArgument('linear_speed',  default_value='0.25',
            description='Forward/backward base speed (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='0.50',
            description='Rotation base speed (rad/s)'),
        DeclareLaunchArgument('command_timeout_s', default_value='3.0',
            description='Safety stop: zero velocity if no command for N seconds'),
        # Visualizer
        DeclareLaunchArgument('render_rate_hz', default_value='5.0',
            description='Visualizer image publish rate (Hz)'),
        # Feature toggles
        DeclareLaunchArgument('enable_speech', default_value='true',
            description='Launch the speech recognition node'),
        DeclareLaunchArgument('enable_viz',    default_value='true',
            description='Launch the mic array visualizer node'),
    ]

    # ── Node: kinect_audio_node ────────────────────────────────────────────
    audio_node = Node(
        package='kinect_ros2',
        executable='kinect_audio',
        name='kinect_audio_node',
        output='screen',
        parameters=[{
            'device_index':  LaunchConfiguration('device_index'),
            'sample_rate':   LaunchConfiguration('sample_rate'),
            'chunk_frames':  LaunchConfiguration('chunk_frames'),
            'channels':      LaunchConfiguration('channels'),
        }],
    )

    # ── Node: ex08_speech_recognition_node (delayed 1.5 s) ───────────────
    speech_node = TimerAction(
        period=1.5,
        actions=[
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
                    'sample_rate':       LaunchConfiguration('sample_rate'),
                    'vad_energy_thresh': LaunchConfiguration('vad_energy_thresh'),
                    'vad_silence_s':     LaunchConfiguration('vad_silence_s'),
                    'linear_speed':      LaunchConfiguration('linear_speed'),
                    'angular_speed':     LaunchConfiguration('angular_speed'),
                    'command_timeout_s': LaunchConfiguration('command_timeout_s'),
                }],
            ),
        ],
    )

    # ── Node: ex09_mic_visualizer_node (delayed 2.0 s) ───────────────────
    viz_node = TimerAction(
        period=2.0,
        actions=[
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
        ],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[kinect_audio] Starting Kinect audio driver …'),
        audio_node,
        speech_node,
        viz_node,
        LogInfo(msg='[kinect_audio] All nodes queued. Audio stack initialising …'),
    ])


# ════════════════════════════════════════════════════════════════════════════
#  SETUP.PY  –  Add these lines to the console_scripts list
# ════════════════════════════════════════════════════════════════════════════
#
#  In kinect_ros2/setup.py, inside entry_points={'console_scripts': [...]},
#  append the following FOUR entries:
#
#    # ── Audio driver ────────────────────────────────────────────────────
#    'kinect_audio = kinect_ros2.kinect_audio_node:main',
#
#    # ── Ex08: Speech Recognition ─────────────────────────────────────
#    'ex08_speech_recognition = '
#        'kinect_ros2.examples.ex08_speech_recognition.'
#        'speech_recognition_node:main',
#
#    # ── Ex09: Mic Array Visualizer ───────────────────────────────────
#    'ex09_mic_visualizer = '
#        'kinect_ros2.examples.ex09_mic_visualizer.'
#        'mic_visualizer_node:main',
#
# ════════════════════════════════════════════════════════════════════════════
#
#  PACKAGE.XML  –  Add these dependencies (if not already present):
#
#    <exec_depend>python3-pyaudio</exec_depend>
#    <exec_depend>python3-speech-recognition</exec_depend>
#    <exec_depend>python3-matplotlib</exec_depend>
#    <!-- vosk is pip-only, no rosdep key; install manually -->
#
# ════════════════════════════════════════════════════════════════════════════
