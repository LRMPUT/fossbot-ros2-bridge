from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fossbot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kamil',
    maintainer_email='k.mlodzikowski@gmail.com',
    description='ROS 2 bridge to the FOSSBot edu robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = fossbot_bridge.bridge_node:main',
            'lidar_node = fossbot_bridge.lidar_node:main',
            'camera_node = fossbot_bridge.camera_node:main',
            'teleop = fossbot_bridge.teleop:main',
            'scan_bearing = fossbot_bridge.scan_bearing:main',
            'gyro_sign_check = fossbot_bridge.gyro_sign_check:main',
            'wheel_calibrate = fossbot_bridge.wheel_calibrate:main',
            'dashboard = fossbot_bridge.dashboard:main',
        ],
    },
)
