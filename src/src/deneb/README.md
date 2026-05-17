
# deneb

One of the three stars of the summer triad. Path
following node responsible for taking in a smoothed
path and following it.
      
This algorithm works by computing the rotation error,
then executing an angular velocity if the error is above
a certain threshold. Otherwise a forward linear velocity
is executed. This has the effect that the robot alternates
bewteen rotating and driving forward as it approaches the
next node in the path. The other algorithms should update
the path as the robot moves towards it, so there should
always be more path to follow.

## subscriptions

| type                           | description                |
|--------------------------------|----------------------------|
| [nav_msgs/Path][9]             | recieve the path to follow |
| [nav_msgs/Odometry][1]         | recieve the robot position |

## publishers

| type                      | description             |
|---------------------------|-------------------------|
| [geometry_msgs/Twist][9]  | output command velocity |
| [nav_msgs/MarkerArray][6] | output visualization    |

## parameters

| name                         | description                                                               | type   |
|------------------------------|---------------------------------------------------------------------------|--------|
| path_in                      | incoming path to follow                                                   | string |
| pos_in                       | incoming position topic                                                   | string |
| vel_out                      | velocity topic to publish output on                                       | string |
| publish_visual               | whether visual information should be published                            | bool   |
| visual_out                   | topic to publish the visualization on                                     | string |
| vel_publish_interval         | time in between velocity publish commands                                 | float  |
| angular_correction_threshold | angular error (radians) at which a rotation is executed                   | float  |
| hit_distance                 | distance from the next path node we consider it 'reached'                 | float  |
| angular_velocity_scale       | speed (radians/sec) the robot will turn                                   | float  |
| linear_velocity_scale        | speed (meters/sec) the robot will drive forward                           | float  |

## transforms

Expects everything in the map frame.

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
