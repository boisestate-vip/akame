# Path Generator
## Overview
This node generates a path for the robot to follow. It will use A\* as the main pathing algorithm and elastic band as the path smoother. 
The path it generates will be used by the [Path Follower](https://github.com/boisestate-vip/akame/blob/main/doc/nodes/path_follower.md) node to navigate.

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
