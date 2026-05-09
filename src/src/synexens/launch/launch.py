
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys
import subprocess

# launch the node using the config file

def generate_launch_description():

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

    return launch.LaunchDescription([
        misao_node
    ])
