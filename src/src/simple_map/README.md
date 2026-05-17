
# simple map

Grid map the robot builds when navigating. Works
by adding the laser scan or point cloud messages
it recieves onto the map and updating the weights
of every square that an obstacle touches. When these
squares exceed a threshold they are treated as obstacles.

## subscriptions

| type                         | description                           |
|------------------------------|---------------------------------------|
| [nav_msgs/Odometry][1]       | accepts current position of the robot |
| [sensor_msgs/LaserScan][2]   | accepts sensor input                  |
| [sensor_msgs/PointCloud2][3] | accepts point cloud input             |
| [std_msgs/Empty][10]         | accepts the reset trigger             |

## publishers

| type                        | description     |
|-----------------------------|-----------------|
| [nav_msgs/OccupancyGrid][5] | map being built |

## parameters

| name                   | description                                                          | type   |
|------------------------|----------------------------------------------------------------------|--------|
| map_out                | topic to publish the map on                                          | string |
| map_publish_interval   | time in seconds between map publishing                               | float  |
| publish_map_frame      | should the map -> odom transform be published                        | bool   |
| frame_publish_interval | time in seconds between map -> odom transform publish                | float  |
| pos_in                 | topic to recieve odometry on                                         | string |
| points_in              | topic to recieve point cloud data on                                 | string |
| scan_in                | topic to recieve laser scan data on                                  | string |
| reset_in               | topic to recieve reset trigger messages on                           | string |
| map_resolution         | size of a grid square in meters                                      | float  |
| map_hit_weight         | value added to a grid square when a obstacle is seen in it           | int    |
| map_miss_weight        | value removed from a grid square when a lidar scan passes through it | int    |
| map_start_weight       | starting value in the grid squares (note squares cap at 100)         | int    |

## transforms

This node expects to be able to transform any incoming sensor data into the odom frame. It
additionally publishes the map -> odom transform as a static 0. Expects odometry in the odom
or map frame.

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
