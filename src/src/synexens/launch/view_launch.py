
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys
import subprocess

# launch the node using the config file

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='synexens_lidar').find('synexens_lidar')
    default_rviz_config_path = os.path.join(pkg_share, 'config/config.rviz');

    # get config file
    config = os.path.join(
            get_package_share_directory('synexens_lidar'),
            'config',
            'synexens.yaml'
    )

    # the node
    misao_node = launch_ros.actions.Node(
            package='synexens_lidar',
            executable='synexens_node',
            parameters=[config]
    )

    # visualization
    rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        rviz_node,
        misao_node
    ])
