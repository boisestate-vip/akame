
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
import os
import sys
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

import logging
logging.root.setLevel(logging.DEBUG)

# launch the node using the config file

def generate_launch_description():

    # get config file
    share_dir = get_package_share_directory('pi_2')
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share_dir, 'params', 'pi2_config.yaml'),
            description='FPath to the ROS2 parameters file to use.'
    )

    synexens_node = launch_ros.actions.Node(
            package='synexens_lidar',
            executable='synexens_node',
            parameters=[parameter_file],
            name='synexens_node',
            namespace='akame',
            output='screen',
    )

    roboclaw_node = launch_ros.actions.Node(
            package='roboclaw_driver',
            executable='roboclaw_driver',
            parameters=[parameter_file],
            name='roboclaw_driver',
            namespace='akame',
            output='screen',
    )

    tracks_node = launch_ros.actions.Node(
            package='go_m8010_6',
            executable='go_m8010_6_driver',
            parameters=[parameter_file],
            name='go_m8010_6_driver',
            namespace='akame',
            output='screen',
    )

    drum_node = launch_ros.actions.Node(
            package='drum_driver',
            executable='drum_driver',
            parameters=[parameter_file],
            name='drum_driver',
            namespace='akame',
            output='screen',
    )

    urdf_package = FindPackageShare(package='bender_5_urdf').find('bender_5_urdf')
    default_urdf_model_path = os.path.join(urdf_package, 'src', 'bender_5.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_urdf_model_path])}],
    )

    return launch.LaunchDescription([
        params_declare,
        DeclareLaunchArgument(name='urdf_model', default_value=default_urdf_model_path, description='Absolute path to robot model file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        synexens_node,
        roboclaw_node,
        tracks_node,
        drum_node,
    ])
