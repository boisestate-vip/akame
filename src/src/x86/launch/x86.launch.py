
import launch 
import launch.event_handlers
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

def generate_launch_description():

    # get the config file
    config = os.path.join(
        get_package_share_directory('x86'),
        'config',
        'x86_config.yaml'
    )

    # the openvr node
    vive_tracker_node = launch_ros.actions.Node(
        package='vive_tracker',
        executable='vive_tracker',
        namespace='vive_tracker',
        name='vive_tracker',
        parameters=[config],
        output='screen',
    )

    # the odom -> base_link publisher
    base_link_node = launch_ros.actions.Node(
        package='vive_to_odom',
        executable='vive_to_odom',
        namespace='akame',
        name='vive_to_odom',
        parameters=[config],
        output='screen',
    )

    # command that will launch steam vr for us
    # note you may have to edit this path if the position of the steam
    # directory is different on your system. The command (for linux) should
    # be the same though, so just try and find steam-apps and copy from there
    steam_vr_process = launch.actions.ExecuteProcess(
        cmd=[[
            '~/.steam/debian-installation/steamapps/common/SteamLinuxRuntime_sniper/run ',
            '~/.steam/debian-installation/steamapps/common/SteamVR/bin/vrstartup.sh'
        ]],
        shell=True
    )

    # unnessary but helpful command that will remove the room setup 
    # window when it is detected during the steam vr launch
    steam_vr_streamline = launch.actions.ExecuteProcess(
        cmd=[[
            'while ! ps -e | grep steamvr_room_se; do sleep 3; done && pkill -9 steamvr_room_se'
        ]],
        shell=True
    )

    # command to execute during the cleanup process
    # kills the steam vr instance running in the background
    def kill_steam_vr(launch_context):
        subprocess.run(['pkill','-9','vrmonitor'])


    return launch.LaunchDescription([
        steam_vr_process,
        steam_vr_streamline,
        vive_tracker_node,
        base_link_node,

        # execute our kill_steam_vr command on shutdown (CTRL-C)
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnShutdown(
                on_shutdown=[
                    launch.actions.OpaqueFunction(
                        function=kill_steam_vr
                    )
                ]
            )
        )
    ])
