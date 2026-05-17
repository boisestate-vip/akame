
# go m8010 6

differential drive controller for the robot tracks.

## subscriptions

| type                     | description                       |
|--------------------------|-----------------------------------|
| [geometry_msgs/Twist][4] | robot linear and angular velocity |

## parameters

| name             | description                                                  | type   |
|------------------|--------------------------------------------------------------|--------|
| cmd_in           | topic to recieve robot linear and angular velocity on        | string |
| left_motor       | serial device of the left motor. Usually a ``/dev/ttyUSBX``  | string |
| right_motor      | serial device of the right motor. Usually a ``/dev/ttyUSBX`` | string |
| left_motor_id    | internal set id of the left unitree motor                    | int    |
| right_motor_id   | internal set id of the right unitree motor                   | int    |
| position_damping | error multiplier for position in the motor PD controllers    | float  |
| velocity_damping | error multiplier for velocity in the motor PD controllers    | float  |


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
[13]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Int32.msg
