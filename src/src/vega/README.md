
# vega

One of the three stars in the summer triad. This is the path generator node.
It is implimented as an A* algorithm with a few modifications. Pathing is
done from the start to the goal. Each obstacle has a radius around it expanded
to keep the robot farther away. Repathing is done on a short interval periodically,
and full repathing is done over a longer interval.

## subscriptions

| type                           | description                           |
|--------------------------------|---------------------------------------|
| [nav_msgs/OccupancyGrid][5]    | map to path over                      |
| [nav_msgs/Odometry][1]         | accepts current position of the robot |
| [geometry_msgs/PoseStamped][2] | accepts the goal position             |
| [std_msgs/Empty][10]           | accepts the reset trigger             |

## publishers

| type                            | description                              |
|---------------------------------|------------------------------------------|
| [nav_msgs/OccupancyGrid][5]     | map after post-processing in visual mode |
| [nav_msgs/Path][9]              | path generated                           |
| [visualization_msgs/Marker][11] | visualization of the algorithm           |

## parameters

| name                   | description                                                                | type   |
|------------------------|----------------------------------------------------------------------------|--------|
| map_in                 | topic to recieve the map on                                                | string |
| pos_in                 | topic to recieve the robot position on                                     | string |
| goal_in                | topic to recieve the goal on                                               | string |
| reset_in               | topic to recieve the reset callback on                                     | string |
| path_out               | topic to publish the path on                                               | string |
| path_publish_interval  | time interval in seconds the path is published on                          | float  |
| visual_out             | topic to publish visualization on                                          | string |
| map_out                | topic to publish the post-processed map on                                 | string |
| map_obstacle_threshold | minimum distance (in meters) the path can be from an obstacle              | float  |
| publish_visual         | whether the visualization should be published                              | bool   |
| map_resolution         | width of the map grid squares in meters                                    | float  |
| map_hit_weight         | threshold at which a map square is considered an obstacle                  | int    |
| map_miss_weight        | arbitrary parameter                                                        | int    |
| map_start_weight       | arbitrary parameter                                                        | int    | 
| repath_range           | distance (in meters) in front of the robot that is repathed every interval | float  |
| full_repath_interval   | time (in seconds) between full repaths                                     | float  |

## transforms

everything in the map frame

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
