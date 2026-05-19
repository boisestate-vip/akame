"""Launch the regolith_driver against the teensy-regolith udev symlink.

Run setup.sh once on the host so /dev/teensy-regolith exists.

The mass topic stays silent until something publishes on /regolith/actuator_pos.
The roboclaw_driver (with actuator_max_counts, actuator_retracted_mm,
actuator_extended_mm calibrated) is the intended source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    poll_hz = LaunchConfiguration('poll_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/teensy-regolith',
            description='Serial device for the regolith controller teensy.',
        ),
        DeclareLaunchArgument(
            'poll_hz',
            default_value='10',
            description='Sensor polling rate in Hz.',
        ),
        Node(
            package='regolith_driver',
            executable='regolith_driver',
            name='regolith_driver',
            output='screen',
            parameters=[{
                'device':  device,
                'poll_hz': poll_hz,
            }],
        ),
    ])
