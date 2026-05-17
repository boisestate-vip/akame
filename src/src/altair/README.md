
# altair

One of the three stars of the summer triad. Path
smoothing node responsible for taking in a raw
path, smoothing it, and outputting the result to
the path follower.    
    
This algorithm internally using the elastic band
algorithm for path smoothing. This algorithm can
be controlled through the parameters given in the
config.

## subscriptions

| type                           | description                |
|--------------------------------|----------------------------|
| [nav_msgs/Odometry][1]         | recieve the robot position |
| [geometry_msgs/PoseStamped][8] | recieve the goal           |
| [nav_msgs/Path][9]             | recieve the path to smooth |
| [nav_msgs/OccupancyGrid][5]    | recieve the map            |
| [std_msgs/Empty][10]           | reset map topic            |

## publishers

| type                      | description          |
|---------------------------|----------------------|
| [nav_msgs/Path][9]        | output smoothed path |
| [nav_msgs/MarkerArray][6] | output visualization |

## parameters

| name                  | description                                                               | type   |
|-----------------------|---------------------------------------------------------------------------|--------|
| map_in                | incoming map topic                                                        | string |
| pos_in                | incoming robot position topic                                             | string |
| pos_in                | incoming goal position topic                                              | string |
| pos_in                | topic path to smooth is being published on                                | string |
| pos_in                | topic reset trigger is tied to                                            | string |
| path_out              | controls the topic the smoothed path is published on                      | string |
| publish_visual        | should the pathing visualization be published?                            | bool   |
| visual_out            | topic to publish the visualization on                                     | string |
| path_publish_interval | time in seconds between path publishing                                   | float  |
| internal_multiplier   | internal force multiplier for the elastic band                            | float  |
| external_multiplier   | external force multiplier for the elastic band                            | float  |
| constraint_multiplier | constraint force multiplier for the elastic band                          | float  |
| distance_cutoff       | maximum distance from an obstace at which the external for is applied     | float  |
| map_height            | score of a map square needed to consider it an obstacle                   | float  |
| bin_width             | unused parameter                                                          | float  |
| max_length            | length of the path used for smoothing                                     | float  |
| hit_distance          | distance a node has to be from an obstacle in order for the path to break | float  |

## transforms

Expects everything in the map frame.

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
