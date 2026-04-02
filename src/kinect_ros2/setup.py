import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kinect_ros2'

setup(
    name=package_name,  # FIXED: was 'kinect_ros2' (missing quotes)
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'config'), glob('config/*')),
	(os.path.join('share', package_name, 'launch'),
		glob('launch/*.launch.py')),
	(os.path.join('share', package_name, 'config'),
		glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaideepchouhan',
    maintainer_email='jaideepchouhan123@gmail.com',
    description='ROS2 Jazzy driver for Kinect 360 v1 using libfreenect',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_driver = kinect_ros2.kinect_driver_node:main',
            'depth_alert = kinect_ros2.depth_alert_node:main',
            'tilt_control = kinect_ros2.tilt_control_node:main',

		# ── Example nodes (v2, new) ─────────────────────────
	    'ex01_rgb_viewer = kinect_ros2.examples.ex01_rgb_viewer.rgb_viewer_node:main',

        ],
    },
)
