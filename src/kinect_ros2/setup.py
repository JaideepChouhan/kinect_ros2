import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kinect_ros2'

setup(
    name=package_name,
    version='2.0.0',  # Update to v2
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaideepchouhan',
    maintainer_email='jaideepchouhan123@gmail.com',
    description='ROS2 Jazzy driver for Kinect 360 v1 using libfreenect + examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_driver = kinect_ros2.kinect_driver_node:main',
            'depth_alert = kinect_ros2.depth_alert_node:main',
            'tilt_control = kinect_ros2.tilt_control_node:main',
            'kinect_audio = kinect_ros2.kinect_audio_node:main',
            'kinect_accel = kinect_ros2.kinect_accel_node:main',
            'depth_to_laserscan = kinect_ros2.depth_to_laserscan_node:main',

            # Example nodes (v2)
            'ex01_rgb_viewer = kinect_ros2.examples.ex01_rgb_viewer.rgb_viewer_node:main',
            'ex02_depth_ruler = kinect_ros2.examples.ex02_depth_ruler.depth_ruler_node:main',
            'ex03_depth_histogram = kinect_ros2.examples.ex03_depth_histogram.depth_histogram_node:main',
            'ex04_hand_detector = kinect_ros2.examples.ex04_hand_detector.hand_detector_node:main',
            'ex05_coloured_cloud = kinect_ros2.examples.ex05_coloured_cloud.coloured_cloud_node:main',
            'ex06_obstacle_avoidance = kinect_ros2.examples.ex06_obstacle_avoidance.obstacle_avoidance_node:main',
            'ex08_mic_visualizer = kinect_ros2.examples.ex08_mic_visualizer.mic_visualizer_node:main',
            'ex09_speech_recognition = kinect_ros2.examples.ex09_speech_recognition.speech_recognition_node:main',
            'ex10_tilt_demo = kinect_ros2.examples.ex10_tilt_demo.ex10_tilt_demo_node:main',
        ],
    },
)
