
# vive to odom

Takes in vive tracker data, publishes the odom -> base_link transform
and the odom data from the base_link reference frame.

## subscriptions

| type                   | description                                                |
|------------------------|------------------------------------------------------------|
| [nav_msgs/Odometry][1] | odometry from a reference frame transformable to base_link |

## publishers

| type                   | description                          |
|------------------------|--------------------------------------|
| [nav_msgs/Odometry][1] | odometry in the odom reference frame |

## parameters

| name                   | description                                                            | type   |
|------------------------|------------------------------------------------------------------------|--------|
| frame_publish_interval | time interval in seconds to publish the odom -> base_link transform on | float  |
| pos_in                 | topic to recieve odometry messages on                                  | string |
| pos_out                | topic to publish transformed odometry on                               | string |

## transforms

This node is responsible for publishing odom -> base_link. This node
expects to be able to transform the incoming odometry frame id to base_link.

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
