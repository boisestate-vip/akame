# Path Follower
## Overview
This node follows the path provided by the [Path Generator](https://github.com/boisestate-vip/akame/blob/main/doc/nodes/path_generator.md).

## Inputs
| Name        | Type                                | Description                                                      |
|-------------|-------------------------------------|------------------------------------------------------------------|
| goal_pose | [geometry_msgs/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html) | Goal pose the path will end at |
| pose_in | [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg) | Current pose of the robot |
| map | [nav_msgs/OccupancyGrid](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg) | Arena map, used to naviagte around obstacles |

## Outputs
| Name        | Type                                | Description                                                      |
|-------------|-------------------------------------|------------------------------------------------------------------|
| path | [nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html) | Obstacle free path to follow |
