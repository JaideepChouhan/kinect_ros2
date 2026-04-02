from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('device_index', default_value='0'),
        LogInfo(msg='[Ex01] Launching RGB Viewer...'),
        Node(
            package='kinect_ros2',
            executable='ex01_rgb_viewer',
            name='rgb_viewer_node',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'device_index': LaunchConfiguration('device_index'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
