
# drum driver

simple node wrapping the unitree go motor sdk for
spinning the regolith drum.

## subscriptions

| type                   | description                 |
|------------------------|-----------------------------|
| [std_msgs/Float64][12] | drum velocity (radians/sec) |

## parameters

| name             | description                                             | type   |
|------------------|---------------------------------------------------------|--------|
| motor_id         | internal id the motor has been set to                   | int    |
| position_damping | multiplier on the positional error                      | float  |
| velocity_damping | multiplier on the velocity error                        | float  |
| cmd_id           | topic to recieve velocity commands on                   | string |
| motor            | serial device the motor is. Normally a ``/dev/ttyUSBX`` | string |

## transforms

not relevant

## _

[1]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
[3]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg
[4]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg
[5]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
[6]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/MarkerArray.msg
[7]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
[8]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
[9]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
[10]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg
[11]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/Marker.msg
[12]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Float64.msg
