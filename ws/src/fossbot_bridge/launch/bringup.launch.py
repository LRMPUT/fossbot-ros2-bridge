"""Bring up the full bridge: robot description, hardware bridge, lidar, rviz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory('fossbot_bridge')
    desc_share = get_package_share_directory('fossbot_description')
    params = os.path.join(bridge_share, 'config', 'bridge.yaml')
    rviz_cfg = os.path.join(desc_share, 'rviz', 'fossbot.rviz')

    robot_host = LaunchConfiguration('robot_host')
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_description = LaunchConfiguration('use_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_host', default_value='fossbotrpi1.local',
            description='Hostname or IP of the robot running fossbot_agent.py'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the lidar node'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Start RViz with the FOSSBot config'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start the camera node'),
        DeclareLaunchArgument(
            'use_description', default_value='true',
            description='Publish the URDF and the TF tree below base_footprint. '
                        'Without it RViz cannot place /scan or the robot model.'),

        # Must come first: everything below base_footprint (wheels, lidar mount,
        # camera) comes from here, and /scan is unrenderable without it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'description.launch.py')),
            condition=IfCondition(use_description),
        ),

        Node(
            package='fossbot_bridge',
            executable='bridge_node',
            name='fossbot_bridge',
            output='screen',
            parameters=[params, {'robot_host': robot_host}],
        ),
        Node(
            package='fossbot_bridge',
            executable='lidar_node',
            name='fossbot_lidar',
            output='screen',
            parameters=[params, {'robot_host': robot_host}],
            condition=IfCondition(use_lidar),
        ),
        Node(
            package='fossbot_bridge',
            executable='camera_node',
            name='fossbot_camera',
            output='screen',
            parameters=[params, {'robot_host': robot_host}],
            condition=IfCondition(use_camera),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
