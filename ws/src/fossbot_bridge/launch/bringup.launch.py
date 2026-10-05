"""Bring up the full bridge: robot description, hardware bridge, lidar, rviz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_share = get_package_share_directory('fossbot_bridge')
    desc_share = get_package_share_directory('fossbot_description')
    default_params = os.path.join(bridge_share, 'config', 'bridge.yaml')
    params = LaunchConfiguration('params_file')
    rviz_cfg = os.path.join(desc_share, 'rviz', 'fossbot.rviz')

    robot_host = ParameterValue(LaunchConfiguration('robot_host'), value_type=str)
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_description = LaunchConfiguration('use_description')
    use_dashboard = LaunchConfiguration('use_dashboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            # FOSSBOT_HOST may carry an ssh user ("user@host") for the robot
            # scripts; the bridge only needs the host part.
            'robot_host',
            default_value=(os.environ.get('FOSSBOT_HOST', '').split('@')[-1] or None),
            description='Hostname or IP of the robot running fossbot_agent.py'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='ROS parameter YAML (defaults to the bundled bridge.yaml)'),
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
            'use_dashboard', default_value='true',
            description='Show the status dashboard (a window, or a text '
                        'summary in this terminal when there is no display)'),
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
            package='fossbot_bridge',
            executable='dashboard',
            name='fossbot_dashboard',
            output='screen',
            parameters=[params],
            condition=IfCondition(use_dashboard),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
