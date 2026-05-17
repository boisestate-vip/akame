
# roboclaw driver

Port of the roboclaw controller library that
works on unix. Accepts a velocity input command
and (in our system) drives a linear actuator.

## subscriptions

| type                 | description                                       |
|----------------------|---------------------------------------------------|
| [std_msgs/Int32][13] | velocity (arbitrary units) to run the actuator at |

## parameters

| name         | description                                                              | type   |
|--------------|--------------------------------------------------------------------------|--------|
| robclaw_port | serial device the roboclaw connects on. Usually through ``/dev/ttyACMX`` | string |
| baud         | baud rate for the controller communication                               | int    |
| cmd_vel in   | topic to recieve our velocity commands on                                | string |
| address      | internal set address of the roboclaw controller                          | int    |

## transforms

does not apply

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
