from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

import logging
logging.root.setLevel(logging.DEBUG)

def generate_launch_description():
    pkg_share = get_package_share_directory('waveshare_demo')
    default_model_path = os.path.join(pkg_share, 'src', 'waveshare.sdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
    )
    map_node = Node(
        package='simple_map',
        executable='simple_map',
        name='simple_map',
        output='screen',
    )
    controller_node = Node(
        package='waveshare_driver',
        executable='waveshare_driver',
        name='waveshare_driver',
        output='screen',
    )
    synexens_node = Node(
        package='synexens_lidar',
        executable='synexens_node',
        name='synexens_node',
        output='screen',
    )
    f710_node = Node(
        package='f710',
        executable='f710',
        name='f710',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_node,
        controller_node,
        synexens_node,
    ])
