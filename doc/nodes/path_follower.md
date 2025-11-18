# Path Follower
## Overview
This node follows the path provided by the [Path Generator](https://github.com/boisestate-vip/akame/blob/main/doc/nodes/path_generator.md).

## Inputs
| Name        | Type                                | Description                                                      |
|-------------|-------------------------------------|------------------------------------------------------------------|
| pose_in | [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg) | Current pose of the robot |
| path | [nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html) | Obstacle free path to follow |

## Outputs
Raw instructions to directly control the differential drive
