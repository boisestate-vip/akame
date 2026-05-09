
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

import logging
logging.root.setLevel(logging.DEBUG)

def generate_launch_description():
    share_dir = get_package_share_directory('pi_1')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'pi1_config.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    front_left_gs5 = LifecycleNode(package='ydlidar_ros2_driver',
                                   executable='ydlidar_ros2_driver_node',
                                   name='front_left_gs5',
                                   output='screen',
                                   emulate_tty=True,
                                   parameters=[parameter_file],
                                   namespace='akame',
                                  )
    front_right_gs5 = LifecycleNode(package='ydlidar_ros2_driver',
                                    executable='ydlidar_ros2_driver_node',
                                    name='front_right_gs5',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[parameter_file],
                                    namespace='akame',
                                   )
    rear_left_gs2 = LifecycleNode(package='ydlidar_ros2_driver',
                                  executable='ydlidar_ros2_driver_node',
                                  name='rear_left_gs2',
                                  output='screen',
                                  emulate_tty=True,
                                  parameters=[parameter_file],
                                  namespace='akame',
                                 )
    rear_right_gs2 = LifecycleNode(package='ydlidar_ros2_driver',
                                   executable='ydlidar_ros2_driver_node',
                                   name='rear_right_gs2',
                                   output='screen',
                                   emulate_tty=True,
                                   parameters=[parameter_file],
                                   namespace='akame',
                                  )

    return LaunchDescription([
        params_declare,
        #front_left_gs5,
        #front_right_gs5,
        rear_left_gs2,
        rear_right_gs2,
    ])
