"""Publish the FOSSBot URDF and the fixed-joint TF tree.

The hardware bridge only publishes odom -> base_footprint. Everything below
base_footprint -- the wheels, the lidar mount, the camera -- comes from here.
Without this node RViz has no transform for lidar_scan_frame, so /scan cannot
be rendered at all even though the topic is clearly arriving.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('fossbot_description'),
                        'urdf', 'fossbot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf]), value_type=str)

    use_jsp = LaunchConfiguration('use_joint_state_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_publisher', default_value='false',
            description='Publish zeroed joint states. Leave false when the '
                        'bridge is running -- it publishes real wheel angles, '
                        'and two publishers on /joint_states fight each other.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(use_jsp),
        ),
    ])
